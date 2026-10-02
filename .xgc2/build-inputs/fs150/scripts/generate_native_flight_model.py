#!/usr/bin/env python3
"""Extract the configured FS150 SDF rigid-body/motor asset and actual overrides.

No tuning defaults or firmware parameter ranges live here. The FCU owner supplies
its one supported-name catalog; absent actual overrides remain absent. Only the
flat model, rigid attachments and parallel +body-Z motor layout used by FS150 are
supported. All inertials are aggregated at their declared SDF reference poses.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
import sys
from pathlib import Path
import xml.etree.ElementTree as ET


class ModelError(ValueError):
    pass


def required(node, tag, location):
    found = node.findall(tag)
    if len(found) != 1:
        raise ModelError('{} requires exactly one {}'.format(location, tag))
    return found[0]


def text(node, tag, location):
    value = required(node, tag, location).text
    if value is None or not value.strip():
        raise ModelError('{} has empty {}'.format(location, tag))
    return value.strip()


def number(value, location, positive=False, nonnegative=False):
    try:
        result = float(value)
    except (TypeError, ValueError):
        raise ModelError('{} is not numeric'.format(location))
    if not math.isfinite(result) or (positive and result <= 0) or (nonnegative and result < 0):
        raise ModelError('{} must be finite{}'.format(location, ' and positive' if positive else ''))
    return result


def integer(value, location):
    if not re.fullmatch(r'[+-]?\d+', value):
        raise ModelError('{} is not an integer'.format(location))
    return int(value)


def vector(value, size, location):
    parts = value.split()
    if len(parts) != size:
        raise ModelError('{} requires {} components'.format(location, size))
    return [number(item, location) for item in parts]


def transpose(matrix):
    return [list(row) for row in zip(*matrix)]


def multiply(left, right):
    return [[math.fsum(left[i][k] * right[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def rotate(rotation, point):
    return [math.fsum(rotation[i][j] * point[j] for j in range(3)) for i in range(3)]


def subtract(left, right):
    return [a - b for a, b in zip(left, right)]


def pose(node, location):
    value = required(node, 'pose', location)
    if (any(value.get(key) for key in ('relative_to', 'frame'))
            or any(key not in ('relative_to', 'frame') for key in value.attrib)):
        raise ModelError('{} uses an unsupported pose frame'.format(location))
    values = vector(value.text or '', 6, location + '/pose')
    x, y, z, roll, pitch, yaw = values
    sr, cr = math.sin(roll), math.cos(roll)
    sp, cp = math.sin(pitch), math.cos(pitch)
    sy, cy = math.sin(yaw), math.cos(yaw)
    # SDF fixed-axis RPY: Rz(yaw) Ry(pitch) Rx(roll), local -> parent.
    rotation = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                [-sp, cp*sr, cp*cr]]
    return [x, y, z], rotation, values


def determinant(m):
    return (m[0][0]*(m[1][1]*m[2][2]-m[1][2]*m[2][1])
            - m[0][1]*(m[1][0]*m[2][2]-m[1][2]*m[2][0])
            + m[0][2]*(m[1][0]*m[2][1]-m[1][1]*m[2][0]))


def check_inertia(matrix, location):
    # Sylvester's criterion on the symmetric, declared tensor, without
    # replacing its coefficients with a diagonal or hand-tuned approximation.
    if (matrix[0][0] <= 0 or matrix[0][0]*matrix[1][1]-matrix[0][1]**2 <= 0
            or determinant(matrix) <= 0):
        raise ModelError('{} is not positive definite'.format(location))


def unique_named(nodes, location):
    result = {}
    for node in nodes:
        name = node.get('name')
        if not name or name in result:
            raise ModelError('{} has missing/duplicate name {}'.format(location, name))
        result[name] = node
    return result


def leaf_values(node, path='sdf'):
    """Preserve all source leaf key/value pairs, including unused channels."""
    result = []
    if not list(node):
        result.append({'path': path, 'text': (node.text or '').strip(), 'attributes': dict(node.attrib)})
    for index, child in enumerate(node):
        suffix = "{}[{}]".format(child.tag, index)
        if 'name' in child.attrib:
            suffix += ':' + child.attrib['name']
        result.extend(leaf_values(child, path + '/' + suffix))
    return result


def parse_overlay(data):
    records = []
    seen = set()
    for line_number, raw in enumerate(data.decode('utf-8').splitlines(), 1):
        row = raw.strip()
        if not row or row.startswith('#'):
            continue
        fields = row.split()
        if len(fields) != 5:
            raise ModelError('parameter line {} requires five QGC fields'.format(line_number))
        vehicle, component, name, value_text, type_text = fields
        if not re.fullmatch(r'[A-Z][A-Z0-9_]*', name) or name in seen:
            raise ModelError('parameter line {} has invalid/duplicate name {}'.format(line_number, name))
        seen.add(name)
        param_type = integer(type_text, name + '/type')
        if param_type == 6:
            value = integer(value_text, name)
            if not -(2**31) <= value < 2**31:
                raise ModelError(name + ' is outside int32 storage')
        elif param_type == 9:
            value = number(value_text, name)
        else:
            raise ModelError('{} has unsupported QGC type {}'.format(name, param_type))
        records.append({'name': name, 'value': value, 'value_text': value_text, 'type': param_type,
                        'vehicle_id': integer(vehicle, name + '/vehicle_id'),
                        'component_id': integer(component, name + '/component_id'), 'line': line_number})
    if not records:
        raise ModelError('parameter overlay is empty')
    return records


def parse_catalog(data):
    catalog = json.loads(data.decode('utf-8'))
    names = catalog.get('names') if isinstance(catalog, dict) else catalog
    if (not isinstance(names, list) or not names or
            any(not isinstance(name, str) or not re.fullmatch(r'[A-Z][A-Z0-9_]*', name) for name in names)
            or len(set(names)) != len(names)):
        raise ModelError('FCU owner catalog requires unique names, without another default/range table')
    return sorted(names)


def source_receipt(path, data):
    return {'path': str(path), 'sha256': hashlib.sha256(data).hexdigest(), 'bytes': len(data)}


def cpp_header(document):
    """The same extracted values, with no ROS/Eigen/default table dependency."""
    def scalar(value):
        if not math.isfinite(value):
            raise ModelError('cannot emit a non-finite C++ asset value')
        result = format(value, '.17g')
        return result if any(character in result for character in '.eE') else result + '.0'

    def vector_literal(values):
        return '{' + ', '.join(scalar(value) for value in values) + '}'

    lines = ['#pragma once', '// Generated from the declared FS150 SDF and actual parameter overlay.',
             '// Body axes: FLU; inertia/rotor positions are about the aggregate COM.',
             '// Contains actual overrides only. The FCU owner supplies defaults/ranges.',
             '#include <array>', 'namespace xgc_lightweight { namespace fs150_native_asset {',
             'inline constexpr double mass = ' + scalar(document['mass']) + ';',
             'inline constexpr double center_of_mass[3] = ' + vector_literal(document['center_of_mass']) + ';',
             'inline constexpr double inertia[3][3] = {' +
             ', '.join(vector_literal(row) for row in document['inertia']) + '};',
             'struct Rotor { int motor_number; double position[3]; double position_from_base_link[3];',
             '  int spin_sign; int reaction_torque_sign; double kf; double km; double moment_constant;',
             '  double tau_up; double tau_down; double max_rot_velocity; double rotor_velocity_slowdown_sim;',
             '  double input_offset; double input_scaling; double zero_position_armed; double zero_position_disarmed; };',
             'inline constexpr std::array<Rotor, ' + str(len(document['rotors'])) + '> rotors{{']
    for rotor in document['rotors']:
        channel = rotor['control_channel']
        values = [str(rotor['motor_number']), vector_literal(rotor['position']),
                  vector_literal(rotor['position_from_base_link']), str(rotor['spin_sign']),
                  str(rotor['reaction_torque_sign'])]
        values.extend(scalar(rotor[key]) for key in ('kf', 'km', 'moment_constant', 'tau_up', 'tau_down',
                                                    'max_rot_velocity', 'rotor_velocity_slowdown_sim'))
        values.extend(scalar(channel[key]) for key in ('input_offset', 'input_scaling',
                                                     'zero_position_armed', 'zero_position_disarmed'))
        lines.append('  {' + ', '.join(values) + '},')
    lines.extend(['}};', 'struct FcuOverride { const char *name; double value; int qgc_type; };',
                  'inline constexpr std::array<FcuOverride, ' + str(len(document['fcu_overrides'])) + '> fcu_overrides{{'])
    types = {entry['name']: entry['type'] for entry in document['parameter_records']}
    for name, value in sorted(document['fcu_overrides'].items()):
        lines.append('  {' + json.dumps(name) + ', ' + scalar(value) + ', ' + str(types[name]) + '},')
    lines.append('}};')
    for key, source in sorted(document['sources'].items()):
        lines.append('inline constexpr char ' + key + '_sha256[] = ' + json.dumps(source['sha256']) + ';')
    lines.extend(['}} // namespace xgc_lightweight::fs150_native_asset', ''])
    return '\n'.join(lines)


def generate(sdf_path, params_path, catalog_path):
    sdf_path, params_path, catalog_path = map(Path, (sdf_path, params_path, catalog_path))
    sdf_bytes, params_bytes, catalog_bytes = (p.read_bytes() for p in (sdf_path, params_path, catalog_path))
    try:
        sdf = ET.fromstring(sdf_bytes)
    except ET.ParseError as error:
        raise ModelError('invalid SDF: ' + str(error))
    models = sdf.findall('model')
    if sdf.tag != 'sdf' or sdf.get('version') != '1.6' or len(models) != 1:
        raise ModelError('requires one direct SDF 1.6 model')
    model = models[0]
    if not model.get('name') or model.findall('model') or model.findall('include'):
        raise ModelError('nested/include/unnamed models are unsupported')
    if text(model, 'static', 'model') not in ('0', 'false'):
        raise ModelError('static model cannot become a native flight model')
    links = unique_named(model.findall('link'), 'links')
    joints = unique_named(model.findall('joint'), 'joints')
    if 'base_link' not in links:
        raise ModelError('model has no base_link body frame')
    base_position, base_rotation, _ = pose(links['base_link'], 'base_link')
    to_body = transpose(base_rotation)
    inertia_terms = []
    for name, link in sorted(links.items()):
        position, rotation, raw_pose = pose(link, 'link ' + name)
        inertial = required(link, 'inertial', name)
        local_com, local_rotation, raw_inertial_pose = pose(inertial, name + '/inertial')
        mass = number(text(inertial, 'mass', name), name + '/mass', positive=True)
        tensor = required(inertial, 'inertia', name)
        values = {key: number(text(tensor, key, name), name + '/' + key)
                  for key in ('ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz')}
        local_tensor = [[values['ixx'], values['ixy'], values['ixz']],
                        [values['ixy'], values['iyy'], values['iyz']],
                        [values['ixz'], values['iyz'], values['izz']]]
        check_inertia(local_tensor, name + '/inertial')
        offset = rotate(rotation, local_com)
        center_model = [position[i] + offset[i] for i in range(3)]
        center_body = rotate(to_body, subtract(center_model, base_position))
        inertial_rotation = multiply(to_body, multiply(rotation, local_rotation))
        body_tensor = multiply(multiply(inertial_rotation, local_tensor), transpose(inertial_rotation))
        inertia_terms.append({'link': name, 'mass': mass, 'center_of_mass': center_body,
                              'inertia_at_link_com': body_tensor, 'pose': raw_pose,
                              'inertial_pose': raw_inertial_pose, 'declared_inertia': values})
    mass = math.fsum(term['mass'] for term in inertia_terms)
    com = [math.fsum(term['mass'] * term['center_of_mass'][i] for term in inertia_terms) / mass
           for i in range(3)]
    inertia = [[math.fsum(term['inertia_at_link_com'][i][j] + term['mass'] *
                         ((sum(d*d for d in subtract(term['center_of_mass'], com)) if i == j else 0)
                          - (term['center_of_mass'][i]-com[i]) * (term['center_of_mass'][j]-com[j]))
                         for term in inertia_terms) for j in range(3)] for i in range(3)]
    check_inertia(inertia, 'aggregate inertia')
    motor_plugins = [p for p in model.findall('plugin') if Path(p.get('filename', '')).name == 'libgazebo_motor_model.so']
    if len(motor_plugins) != 4:
        raise ModelError('requires four explicit FS150 motor plugins')
    plugins = unique_named(motor_plugins, 'motor plugins')
    used_joints, used_links, motors = set(), set(), {}
    for name, plugin in sorted(plugins.items()):
        link_name, joint_name = text(plugin, 'linkName', name), text(plugin, 'jointName', name)
        motor_number = integer(text(plugin, 'motorNumber', name), name + '/motorNumber')
        if motor_number in motors or link_name in used_links or joint_name in used_joints:
            raise ModelError('duplicate motor number/link/joint')
        used_links.add(link_name)
        used_joints.add(joint_name)
        if link_name not in links or joint_name not in joints:
            raise ModelError(name + ' references an absent link/joint')
        joint = joints[joint_name]
        if (joint.get('type') != 'revolute' or text(joint, 'parent', joint_name) != 'base_link'
                or text(joint, 'child', joint_name) != link_name):
            raise ModelError(name + ' has unsupported motor joint topology')
        axis = required(joint, 'axis', joint_name)
        if required(axis, 'xyz', joint_name).attrib:
            raise ModelError(name + ' uses an unsupported rotor axis reference')
        if text(axis, 'use_parent_model_frame', joint_name) not in ('1', 'true'):
            raise ModelError(name + ' axis must explicitly use parent model frame')
        joint_axis = vector(text(axis, 'xyz', joint_name), 3, joint_name)
        norm = math.sqrt(sum(value*value for value in joint_axis))
        if norm <= 0:
            raise ModelError(name + ' has zero rotor axis')
        body_axis = rotate(to_body, [value / norm for value in joint_axis])
        link_position, link_rotation, _ = pose(links[link_name], link_name)
        force_axis = rotate(multiply(to_body, link_rotation), [0.0, 0.0, 1.0])
        if any(abs(a - b) > 1e-9 for actual in (body_axis, force_axis)
               for a, b in zip(actual, (0.0, 0.0, 1.0))):
            raise ModelError(name + ' has unsupported non-parallel/tilted rotor axes')
        spin = text(plugin, 'turningDirection', name)
        if spin not in ('cw', 'ccw'):
            raise ModelError(name + ' has unsupported turningDirection')
        def positive(key):
            return number(text(plugin, key, name), name + '/' + key, positive=True)
        kf, moment = positive('motorConstant'), positive('momentConstant')
        position_body = rotate(to_body, subtract(link_position, base_position))
        motors[motor_number] = {'motor_number': motor_number, 'plugin': name, 'link': link_name,
                               'joint': joint_name, 'position': subtract(position_body, com),
                               'position_from_base_link': position_body, 'axis': body_axis,
                               'spin': spin, 'spin_sign': 1 if spin == 'ccw' else -1,
                               'reaction_torque_sign': -1 if spin == 'ccw' else 1,
                               'kf': kf, 'km': kf * moment, 'moment_constant': moment,
                               'tau_up': positive('timeConstantUp'), 'tau_down': positive('timeConstantDown'),
                               'max_rot_velocity': positive('maxRotVelocity'),
                               'rotor_velocity_slowdown_sim': positive('rotorVelocitySlowdownSim')}
    if set(motors) != {0, 1, 2, 3}:
        raise ModelError('motorNumber must be exactly 0,1,2,3')
    # All remaining links must be rigidly attached: no missing independent
    # body, free servo or unlocked IMU joint is silently folded into one tensor.
    attached = {'base_link'}
    for name, joint in joints.items():
        if name in used_joints:
            attached.add(text(joint, 'child', name))
            continue
        parent, child = text(joint, 'parent', name), text(joint, 'child', name)
        if parent != 'base_link' or child not in links:
            raise ModelError(name + ' has unsupported rigid attachment topology')
        if joint.get('type') != 'fixed':
            if joint.get('type') != 'revolute':
                raise ModelError(name + ' has unsupported joint type')
            limit = required(required(joint, 'axis', name), 'limit', name)
            lower = number(text(limit, 'lower', name), name + '/lower')
            upper = number(text(limit, 'upper', name), name + '/upper')
            if lower != 0 or upper != 0:
                raise ModelError(name + ' is not locked at its declared reference pose')
        if child in attached:
            raise ModelError('link ' + child + ' has multiple parents')
        attached.add(child)
    if set(links) != attached:
        raise ModelError('link without supported rigid/motor attachment')
    mavlink = [p for p in model.findall('plugin') if Path(p.get('filename', '')).name == 'libgazebo_mavlink_interface.so']
    if len(mavlink) != 1:
        raise ModelError('requires one explicit motor control_channels source')
    channels = required(mavlink[0], 'control_channels', 'mavlink_interface')
    by_index, all_channels = {}, []
    for name, channel in sorted(unique_named(channels.findall('channel'), 'control channels').items()):
        index = integer(text(channel, 'input_index', name), name + '/input_index')
        if index < 0 or index in by_index:
            raise ModelError('duplicate/negative control channel input_index')
        record = {'name': name, 'input_index': index, 'bound_motor_number': index if index in motors else None,
                  'input_offset': number(text(channel, 'input_offset', name), name + '/input_offset'),
                  'input_scaling': number(text(channel, 'input_scaling', name), name + '/input_scaling', positive=True),
                  'zero_position_armed': number(text(channel, 'zero_position_armed', name), name + '/zero_position_armed'),
                  'zero_position_disarmed': number(text(channel, 'zero_position_disarmed', name), name + '/zero_position_disarmed'),
                  'joint_control_type': text(channel, 'joint_control_type', name)}
        by_index[index] = record
        all_channels.append(record)
    for index, motor in sorted(motors.items()):
        if index not in by_index or by_index[index]['joint_control_type'] != 'velocity':
            raise ModelError('motor {} lacks a velocity control channel'.format(index))
        channel = by_index[index]
        if channel['zero_position_armed'] < 0 or channel['zero_position_disarmed'] < 0:
            raise ModelError('motor zero positions must be nonnegative angular speeds')
        motor['control_channel'] = channel
    records = parse_overlay(params_bytes)
    names = parse_catalog(catalog_bytes)
    actual = {record['name']: record['value'] for record in records}
    overrides = {name: actual[name] for name in names if name in actual}
    return {'schema': 'xgc2.native_flight_model.v1', 'model_name': model.get('name'),
            'frame': {'body_axes': 'FLU', 'body_frame': 'base_link', 'center_of_mass_origin': 'base_link',
                      'rotor_position_origin': 'aggregate center of mass', 'inertia_origin': 'aggregate center of mass',
                      'aggregation': 'all declared inertials at SDF reference link/rotor poses'},
            'units': {'mass': 'kg', 'center_of_mass': 'm', 'inertia': 'kg*m^2', 'position': 'm',
                      'kf': 'N/(rad/s)^2', 'km': 'N*m/(rad/s)^2', 'moment_constant': 'm',
                      'tau_up': 's', 'tau_down': 's', 'max_rot_velocity': 'rad/s',
                      'input_scaling': 'rad/s per normalized input', 'zero_position_armed': 'rad/s',
                      'zero_position_disarmed': 'rad/s'},
            'mass': mass, 'center_of_mass': com, 'inertia': inertia, 'links': inertia_terms,
            'rotors': [motors[index] for index in sorted(motors)], 'control_channels': all_channels,
            'fcu_overrides': overrides, 'fcu_unprovided_names': [name for name in names if name not in actual],
            'fcu_excluded_source_names': sorted(set(actual) - set(names)), 'parameter_records': records,
            'sdf_leaf_values': leaf_values(sdf),
            'sources': {'sdf': source_receipt(sdf_path, sdf_bytes), 'parameters': source_receipt(params_path, params_bytes),
                        'fcu_name_catalog': source_receipt(catalog_path, catalog_bytes)}}


def main():
    root = Path(__file__).resolve().parent.parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sdf', type=Path, default=root / 'models/fs150/iris.sdf')
    parser.add_argument('--params', type=Path, default=root / 'config/generated/fs150-sitl.params')
    parser.add_argument('--fcu-name-catalog', type=Path, required=True,
                        help='JSON names array or {"names": [...]} exported by the one FCU parameter owner')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--cpp-output', type=Path, help='optional C++17 header of the same extracted values')
    args = parser.parse_args()
    try:
        document = generate(args.sdf, args.params, args.fcu_name_catalog)
        output = json.dumps(document, sort_keys=True, ensure_ascii=False, allow_nan=False, indent=2) + '\n'
        header = cpp_header(document) if args.cpp_output else None
    except (ModelError, OSError, ValueError) as error:
        print('generate_native_flight_model: ' + str(error), file=sys.stderr)
        return 1
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(output, encoding='utf-8')
    if args.cpp_output:
        args.cpp_output.parent.mkdir(parents=True, exist_ok=True)
        args.cpp_output.write_text(header, encoding='utf-8')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
