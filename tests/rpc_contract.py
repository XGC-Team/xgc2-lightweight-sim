"""Check the production Unix management contract without ROS."""
import argparse
import itertools
import json
from pathlib import Path
import subprocess
import tempfile
import time

from native_http import UnixHTTP


class Management:
    def __init__(self, socket_path, instance_id):
        self.socket_path = socket_path
        self.instance_id = instance_id
        self.sequence = itertools.count(1)

    def http(self, method, path, body=None, encoded=None):
        connection = UnixHTTP(self.socket_path)
        try:
            payload = encoded if encoded is not None else (
                None if body is None else json.dumps(body))
            connection.request(method, path, payload, {'Content-Type': 'application/json'})
            response = connection.getresponse()
            result = json.loads(response.read())
            assert result['instance_id'] == self.instance_id, result
            return response.status, result, dict(response.getheaders())
        finally:
            connection.close()

    def get(self, path):
        code, result, _ = self.http('GET', path)
        assert code == 200, (path, code, result)
        return result

    def malformed_length(self, value):
        connection = UnixHTTP(self.socket_path)
        try:
            payload = json.dumps(self.body()).encode()
            connection.putrequest('POST', '/pause')
            connection.putheader('Content-Type', 'application/json')
            connection.putheader('Content-Length', value(len(payload)))
            connection.endheaders(payload)
            response = connection.getresponse()
            result = json.loads(response.read())
            assert response.status == 400, (response.status, result)
        finally:
            connection.close()

    def body(self, **fields):
        result = {'instance_id': self.instance_id,
                  'request_id': 'rpc-' + str(next(self.sequence))}
        result.update(fields)
        return result

    def receipt(self, request_id):
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            result = self.get('/requests/' + request_id)
            assert result['request_id'] == request_id and result['accepted'] is True, result
            if result['phase'] in ('applied', 'cancelled', 'failed'):
                return result
            assert result['phase'] in ('accepted', 'executing'), result
            time.sleep(.005)
        raise AssertionError(('receipt deadline', request_id))

    def mutate(self, path, body, method='POST'):
        code, admitted, _ = self.http(method, path, body)
        assert code == 202 and admitted['request_id'] == body['request_id'], (code, admitted)
        return self.receipt(body['request_id'])

    def projection(self):
        status = self.get('/status')
        return (status['paused'], status['simulation_time_ns'], status['steps'],
                self.get('/entities')['entities'])

    def rejected(self, path, fields, method='POST', code=400):
        body = self.body(**fields)
        status, result, _ = self.http(method, path, body)
        assert status == code, (method, path, body, status, result)
        assert 'error' in result, result
        assert self.http('GET', '/requests/' + body['request_id'])[0] == 404, body


def check(api, config):
    capabilities = api.get('/capabilities')
    assert capabilities['rpc_version'] == 1 and capabilities['ros'] is False, capabilities
    assert type(capabilities['gpu']) is bool, capabilities
    assert set(capabilities['robot_kinds']) == {'fs150', 'scout', 'mecanum'}, capabilities
    groups = {'localization', 'local', 'imu', 'imu_raw', 'state', 'extended_state', 'target'}
    assert set(capabilities['telemetry_groups']) == groups, capabilities
    endpoints = {item['path']: item['methods'] for item in capabilities['endpoints']}
    expected = {'/capabilities': ['GET'], '/config': ['GET'], '/status': ['GET'],
                '/entities': ['GET', 'POST'], '/entities/<id>': ['DELETE'],
                '/entities/<id>/provider': ['POST'], '/pause': ['POST'],
                '/resume': ['POST'], '/step': ['POST'], '/reset': ['POST'],
                '/telemetry-rates': ['GET', 'POST'], '/requests/<request_id>': ['GET']}
    assert endpoints == expected, endpoints
    assert capabilities['limits']['request_bytes'] == 1048576, capabilities
    assert capabilities['limits']['receipt_ttl_ms'] == 300000, capabilities
    assert set(capabilities['sensor_modes']['cpu']) == {'raycast', 'penetrating', 'depth'}
    assert capabilities['sensor_modes']['gpu'] == (['lidar_scan'] if capabilities['gpu'] else [])

    discovered = api.get('/config')
    expected_world = {key: config[key] for key in (
        'epoch_ns', 'model_step_ns', 'output_period_ns', 'input_poll_ns',
        'max_model_step_ns', 'catchup_batch', 'sensor_workers', 'publish_workers',
        'publish_clock')}
    assert discovered['world'] == expected_world, discovered
    assert discovered['telemetry'] == api.get('/telemetry-rates'), discovered
    assert set(discovered['telemetry']['requested_rates_hz']) == groups, discovered
    assert discovered['telemetry']['requested_rates_hz']['imu'] == 42, discovered
    assert discovered['telemetry']['snapshot_cap_hz'] == 125, discovered
    frozen = api.projection()
    assert frozen[:3] == (True, config['epoch_ns'], 0), frozen

    entity_id = frozen[3][0]['id']
    provider_path = '/entities/' + str(entity_id) + '/provider'
    wrong_methods = [('/pause', 'DELETE', ['POST']),
                     ('/resume', 'DELETE', ['POST']),
                     ('/step', 'DELETE', ['POST']),
                     ('/reset', 'DELETE', ['POST']),
                     ('/config', 'POST', ['GET']),
                     ('/capabilities', 'POST', ['GET']),
                     ('/status', 'POST', ['GET']),
                     (provider_path, 'DELETE', ['POST']),
                     ('/entities', 'PUT', ['GET', 'POST'])]
    for path, method, allowed in wrong_methods:
        body = api.body(steps=1, entity_id=entity_id, generation=0)
        code, result, headers = api.http(method, path, body)
        assert code == 405 and result['allowed_methods'] == allowed, (code, result)
        assert [value.strip() for value in headers['Allow'].split(',')] == allowed, headers
        assert api.http('GET', '/requests/' + body['request_id'])[0] == 404
    assert api.http('POST', '/not-an-endpoint')[0] == 404
    assert api.http('POST', '/entities/no-id/provider')[0] == 404
    assert api.projection() == frozen

    invalid_unsigned = (-1, 1.5, '1', None, True, 18446744073709551616)
    for value in invalid_unsigned:
        api.rejected('/step', {'steps': value})
        api.rejected(provider_path, {'generation': value, 'action': 'start'})
        api.rejected('/entities/' + str(entity_id), {'generation': value}, method='DELETE')
        api.rejected('/reset', {'entity_id': entity_id, 'generation': value})
        api.rejected('/reset', {'entity_id': value, 'generation': 0})
        api.rejected('/pause', {'timeout_ms': value})
    for value in (0, 5001, 18446744073709551615):
        api.rejected('/pause', {'timeout_ms': value})
    api.rejected('/step', {'steps': 0})
    api.rejected('/reset', {'generation': 0})
    api.rejected('/reset', {'entity_id': entity_id})
    for path, fields in (('/pause', {'extra': True}),
                         ('/step', {'steps': 1, 'extra': True}),
                         ('/reset', {'entity_id': entity_id, 'generation': 0, 'extra': True}),
                         (provider_path, {'generation': 0, 'action': 'start', 'extra': True}),
                         ('/telemetry-rates', {'rates_hz': {'state': 2}, 'extra': True}),
                         ('/entities', {'entity': {'name': 'extra', 'kind': 'scout'}, 'extra': True})):
        api.rejected(path, fields)
    for value in ('', 'a' * 129, 'a/b', 'a?b', 'a%b', 'a#b', 'a b', '中文'):
        code, result, _ = api.http('POST', '/pause', api.body(request_id=value))
        assert code == 400, (value, code, result)
    for malformed in (lambda size: str(size) + 'junk',
                      lambda size: '+' + str(size),
                      lambda size: str(size) + ' 0'):
        api.malformed_length(malformed)
    for payload in ('null', '[]', '42'):
        code, result, _ = api.http('POST', '/pause', encoded=payload)
        assert code == 400, (payload, code, result)
    duplicate = '{"instance_id":"rpc-contract","request_id":"duplicate-top","request_id":"duplicate-top"}'
    assert api.http('POST', '/pause', encoded=duplicate)[0] == 400
    duplicate = ('{"instance_id":"rpc-contract","request_id":"duplicate-nested",'
                 '"rates_hz":{"state":1,"state":2}}')
    assert api.http('POST', '/telemetry-rates', encoded=duplicate)[0] == 400
    assert api.http('GET', '/requests/duplicate-top')[0] == 404
    assert api.http('GET', '/requests/duplicate-nested')[0] == 404
    assert api.projection() == frozen

    body = api.body(request_id='safe.ID_1:case-2')
    terminal = api.mutate('/pause', body)
    assert terminal['phase'] == 'applied' and terminal['result']['success'], terminal
    code, replay, _ = api.http('POST', '/pause', body)
    assert code == 200 and replay == terminal, (code, replay, terminal)
    assert api.http('POST', '/resume', body)[0] == 409
    foreign = api.body(instance_id='different-world')
    assert api.http('POST', '/pause', foreign)[0] == 409

    started_body = api.body(generation=0, action='start', timeout_ms=5000)
    started = api.mutate(provider_path, started_body)
    result = started['result']
    assert started['phase'] == 'applied' and result['applied'] and result['success'], started
    assert (result['entity_id'], result['generation'], result['enabled'], result['reason']) == (
        entity_id, 1, True, 0), result
    code, replay, _ = api.http('POST', provider_path, started_body)
    assert code == 200 and replay == started
    assert api.http('POST', provider_path, {**started_body, 'action': 'stop'})[0] == 409
    repeated = api.mutate(provider_path, api.body(generation=0, action='start'))['result']
    assert repeated['success'] and repeated['generation'] == 1, repeated
    stale = api.mutate(provider_path, api.body(generation=0, action='stop'))['result']
    assert stale['applied'] and not stale['success'] and stale['reason'] == 1, stale
    assert stale['enabled'] and stale['generation'] == 1, stale
    stopped = api.mutate(provider_path, api.body(generation=1, action='stop'))['result']
    assert stopped['success'] and not stopped['enabled'] and stopped['generation'] == 1

    rate_body = api.body(rates_hz={'state': 2})
    code, rates_receipt, _ = api.http('POST', '/telemetry-rates', rate_body)
    assert code == 200 and rates_receipt['phase'] == 'applied', (code, rates_receipt)
    assert api.get('/requests/' + rate_body['request_id']) == rates_receipt
    code, replay, _ = api.http('POST', '/telemetry-rates', rate_body)
    assert code == 200 and replay == rates_receipt, (code, replay)
    assert api.http('POST', '/pause', api.body(request_id=rate_body['request_id']))[0] == 409
    assert api.http('POST', '/telemetry-rates', api.body(
        request_id=started_body['request_id'], rates_hz={'state': 3}))[0] == 409
    current = api.get('/config')
    assert current['world'] == expected_world, current
    assert current['telemetry'] == api.get('/telemetry-rates')
    assert current['telemetry']['requested_rates_hz']['state'] == 2, current
    assert api.get('/status')['simulation_time_ns'] == config['epoch_ns']

    manual = api.mutate('/step', api.body(steps=2))['result']
    assert manual['success'] and manual['simulation_time_ns'] == config['epoch_ns'] + 4000000
    api.mutate('/resume', api.body())
    body = api.body()
    code, _, headers = api.http('DELETE', '/pause', body)
    assert code == 405 and headers['Allow'] == 'POST', (code, headers)
    denied_step = api.mutate('/step', api.body(steps=1))['result']
    assert denied_step['applied'] and not denied_step['success'] and denied_step['reason'] == 2
    deadline = time.monotonic() + 1
    while api.get('/status')['simulation_time_ns'] <= manual['simulation_time_ns']:
        assert time.monotonic() < deadline, 'wrong-method pause stopped the world'
        time.sleep(.005)
    assert api.get('/status')['paused'] is False
    api.mutate('/pause', api.body())


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--xsim', required=True)
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix='xsim-rpc-contract-') as directory:
        root = Path(directory)
        config = {'instance_id': 'rpc-contract', 'epoch_ns': 1700000000000000000,
                  'paused': True, 'model_step_ns': 2000000, 'output_period_ns': 8000000,
                  'input_poll_ns': 3000000, 'max_model_step_ns': 10000000,
                  'catchup_batch': 4, 'sensor_workers': 1, 'publish_clock': True,
                  'publish_workers': 3,
                  'telemetry_rates_hz': {'imu': 42},
                  'entities': [{'name': kind, 'kind': kind} for kind in ('fs150', 'scout', 'mecanum')]}
        config_path = root / 'world.json'
        config_path.write_text(json.dumps(config))
        socket_path = str(root / 'world.sock')
        with (root / 'server.log').open('w') as log:
            process = subprocess.Popen([args.xsim, '--config', str(config_path), '--socket', socket_path],
                                       stdout=log, stderr=log)
            try:
                api = Management(socket_path, config['instance_id'])
                deadline = time.monotonic() + 5
                while True:
                    assert process.poll() is None, (root / 'server.log').read_text()
                    assert time.monotonic() < deadline, 'native socket not ready'
                    try:
                        api.get('/status')
                        break
                    except (FileNotFoundError, ConnectionRefusedError):
                        time.sleep(.005)
                check(api, config)
            finally:
                process.terminate()
                process.wait(timeout=10)
            assert process.returncode == 0, (root / 'server.log').read_text()
            assert not Path(socket_path).exists(), 'owned native socket survived exit'
    print('PASS: RPC discovery, read-only configuration, methods, validation, and provider receipts')


if __name__ == '__main__':
    main()
