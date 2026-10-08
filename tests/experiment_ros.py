#!/usr/bin/env python3
"""Frozen Experiment bootstrap against an isolated ROS master and native UDS."""
import argparse
import json
import os
import signal
import socket
import subprocess
import threading
import time
import xmlrpc.client
from pathlib import Path
from native_http import NativeProvider, UnixHTTP, assert_retired_ros_surfaces_absent


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--xsim', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    root = Path(args.output)
    root.mkdir(parents=True, exist_ok=True)
    with socket.socket() as free:
        free.bind(('127.0.0.1', 0))
        port = free.getsockname()[1]
    os.environ.update(ROS_MASTER_URI=f'http://127.0.0.1:{port}', ROS_IP='127.0.0.1',
                      ROS_HOME=str(root / 'ros-home'), ROS_LOG_DIR=str(root / 'ros-log'))
    children, logs = [], []

    def spawn(command, name):
        log = open(root / name, 'w')
        logs.append(log)
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        children.append(process)
        return process

    def until(predicate, label, seconds=8):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            try:
                result = predicate()
                if result:
                    return result
            except (OSError, xmlrpc.client.Error):
                pass
            time.sleep(.01)
        raise AssertionError('timeout ' + label)

    try:
        spawn(['roscore', '-p', str(port)], 'roscore.log')
        master = xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'])
        until(lambda: master.getPid('/xsim_experiment_fixture')[0] == 1, 'private master')
        import rospy
        from geometry_msgs.msg import PoseStamped, Twist
        from rosgraph_msgs.msg import Clock
        rospy.init_node('xsim_experiment_fixture', disable_signals=True)
        fixture = Path(__file__).parent / 'experiment-fixtures'
        bootstrap = json.loads((fixture / 'default-mixed.input.json').read_text())
        expected = json.loads((fixture / 'default-mixed.expected-config.json').read_text())
        bootstrap['context']['simulation']['rosMasterUri'] = os.environ['ROS_MASTER_URI']
        filename = root / 'experiment.json'
        filename.write_text(json.dumps(bootstrap))
        socket_path = str(root / 'runtime' / 'world.sock')
        server = spawn([args.xsim, '--experiment-file', str(filename), '--socket', socket_path,
                        '--scene-file', ''], 'xsim.log')
        until(lambda: Path(socket_path).exists() and server.poll() is None, 'native socket')

        def get(path):
            connection = UnixHTTP(socket_path)
            try:
                connection.request('GET', path)
                response = connection.getresponse()
                assert response.status == 200
                return json.loads(response.read())
            finally:
                connection.close()

        configured = get('/config')
        assert configured['instance_id'] == bootstrap['instanceId']
        for key in ('epoch_ns', 'model_step_ns', 'output_period_ns', 'input_poll_ns', 'publish_clock'):
            assert configured['world'][key] == expected[key], (key, configured)
        serial = 0

        def mutate(path, **body):
            nonlocal serial
            serial += 1
            request_id = 'experiment-' + str(serial)
            payload = {'instance_id': bootstrap['instanceId'], 'request_id': request_id, **body}
            connection = UnixHTTP(socket_path)
            try:
                connection.request('POST', path, json.dumps(payload), {'Content-Type': 'application/json'})
                response = connection.getresponse()
                receipt = json.loads(response.read())
                assert response.status == 202, receipt
            finally:
                connection.close()
            result = until(lambda: get('/requests/' + request_id) if get('/requests/' + request_id)['phase'] in
                           ('applied', 'failed', 'cancelled') else None, path)
            assert result['phase'] == 'applied' and result['result']['success'], result
            return result['result']

        latest, lock, subscriptions = {}, threading.Lock(), []

        def subscribe(key, topic, message_type):
            def callback(message):
                with lock:
                    latest[key] = message
            subscriptions.append(rospy.Subscriber(topic, message_type, callback, queue_size=32))

        def sample(key):
            with lock:
                return latest.get(key)

        subscribe('clock', '/clock', Clock)
        providers, ground = {}, []
        for entity in expected['entities']:
            name = entity['name']
            subscribe(name, entity['ros']['localization_pose_topic'], PoseStamped)
            providers[name] = NativeProvider(socket_path, name)
            before = providers[name].read()
            assert not before['enabled'] and before['generation'] == 0
            started = providers[name].start(0)
            assert started['success'] and started['enabled'] and started['generation'] == 1
            if entity['kind'] != 'fs150':
                ground.append(name)
        epoch_ns = int(bootstrap['epochNs'])
        until(lambda: sample('clock') and sample('clock').clock.to_nsec() >= epoch_ns, 'actual /clock')
        mutate('/pause')
        stamp = get('/status')['simulation_time_ns']
        report = {'epochNs': epoch_ns, 'instanceId': bootstrap['instanceId'], 'configurationExact': True,
                  'models': {}, 'privateMaster': os.environ['ROS_MASTER_URI']}
        for entity in expected['entities']:
            name = entity['name']
            pose = until(lambda: sample(name) if sample(name) and sample(name).header.stamp.to_nsec() == stamp else None,
                         name + ' actual localization')
            position = [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
            assert max(abs(a - b) for a, b in zip(position, entity['position'])) < 8e-7 + 1e-12
            report['models'][name] = {'pose': position, 'stampNs': pose.header.stamp.to_nsec(),
                                     'providerGeneration': 1, 'kind': entity['kind']}
        state = master.getSystemState('/xsim_experiment_fixture')
        assert state[0] == 1
        report['graph'] = assert_retired_ros_surfaces_absent(state[2], providers, ground)
        publishers = [rospy.Publisher('/' + name + '/cmd_vel', Twist, queue_size=1) for name in ground]
        for publisher in publishers:
            until(lambda: publisher.get_num_connections() > 0, 'original ground command subscriber')
            command = Twist()
            command.linear.x = .2
            publisher.publish(command)
        time.sleep(.05)
        stepped = mutate('/step', steps=200)['simulation_time_ns']
        assert stepped == stamp + 200 * expected['model_step_ns']
        for entity in expected['entities']:
            if entity['name'] not in ground:
                continue
            name = entity['name']
            pose = until(lambda: sample(name) if sample(name) and sample(name).header.stamp.to_nsec() == stepped else None,
                         name + ' stepped actual localization')
            assert pose.pose.position.x > entity['position'][0] + .01
            report['models'][name]['afterCommandPose'] = [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
        for name, provider in providers.items():
            stopped = provider.stop(provider.read()['generation'])
            assert stopped['success'] and not stopped['enabled']
            assert not provider.read()['enabled']
        server.send_signal(signal.SIGTERM)
        assert server.wait(timeout=5) == 0
        assert not Path(socket_path).exists()
        report.update(actualClock=True, originalCommandObserved=True, normalStop=True, ownedSocketGone=True)
        (root / 'receipt.json').write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(report))
    finally:
        for process in reversed(children):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGTERM)
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)
        for log in logs:
            log.close()


if __name__ == '__main__':
    main()
