"""Exercise the production native server with no ROS runtime or master."""
import argparse
import json
from pathlib import Path
import subprocess
import tempfile
import time

from native_http import NativeProvider, UnixHTTP


def http(socket_path, method, path, body=None):
    connection = UnixHTTP(socket_path)
    try:
        connection.request(method, path, None if body is None else json.dumps(body),
                           {'Content-Type': 'application/json'})
        response = connection.getresponse()
        return response.status, json.loads(response.read())
    finally:
        connection.close()


def receipt(socket_path, request_id):
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        code, result = http(socket_path, 'GET', '/requests/' + request_id)
        assert code == 200, (code, result)
        if result['phase'] in ('applied', 'cancelled', 'failed'):
            assert result['phase'] == 'applied', result
            return result['result']
        time.sleep(.005)
    raise AssertionError('native request has no terminal receipt')


def telemetry_rates(socket_path, config):
    defaults = {'localization': 125, 'local': 30, 'imu': 30, 'imu_raw': 30,
                'state': 1, 'extended_state': 1, 'target': 10}
    code, initial = http(socket_path, 'GET', '/telemetry-rates')
    assert code == 200 and initial['instance_id'] == config['instance_id'], initial
    assert initial['requested_rates_hz'] == defaults, initial
    assert initial['snapshot_period_ns'] == 8000000 and initial['snapshot_cap_hz'] == 125, initial
    assert initial['effective_cap_hz'] == defaults, initial
    before = http(socket_path, 'GET', '/status')[1]

    body = {'instance_id': config['instance_id'], 'request_id': 'rates-first',
            'rates_hz': {'localization': 500, 'local': 15, 'imu_raw': 0}}
    expected = {**defaults, **body['rates_hz']}
    code, applied = http(socket_path, 'POST', '/telemetry-rates', body)
    assert code == 200 and applied['phase'] == 'applied', applied
    assert applied['result']['success'] and applied['result']['requested_rates_hz'] == expected, applied
    assert http(socket_path, 'GET', '/requests/rates-first') == (200, applied)
    current = http(socket_path, 'GET', '/telemetry-rates')[1]
    assert current['requested_rates_hz'] == expected, current
    assert current['effective_cap_hz'] == {**expected, 'localization': 125}, current

    next_body = {'instance_id': config['instance_id'], 'request_id': 'rates-next',
                 'rates_hz': {'local': 20}}
    code, next_applied = http(socket_path, 'POST', '/telemetry-rates', next_body)
    assert code == 200 and next_applied['result']['requested_rates_hz'] == {**expected, 'local': 20}, next_applied
    # Replay reconciles the first application; it must not overwrite newer settings.
    assert http(socket_path, 'POST', '/telemetry-rates', body) == (200, applied)
    assert http(socket_path, 'GET', '/requests/rates-first') == (200, applied)
    assert http(socket_path, 'GET', '/telemetry-rates')[1]['requested_rates_hz']['local'] == 20
    conflict = {**body, 'rates_hz': {'imu': 60}}
    assert http(socket_path, 'POST', '/telemetry-rates', conflict)[0] == 409
    assert http(socket_path, 'POST', '/pause', {'instance_id': config['instance_id'],
                                             'request_id': 'rates-first'})[0] == 409
    assert http(socket_path, 'POST', '/telemetry-rates', {**body, 'instance_id': 'foreign'})[0] == 409

    stable = http(socket_path, 'GET', '/telemetry-rates')[1]
    for index, invalid in enumerate((None, [], {'unknown': 10}, {'imu': True},
                                     {'imu': '30'}, {'imu': None}, {'imu': -1},
                                     {'imu': 1001}, {'imu': 1e-20},
                                     {'local': 15, 'localization': float('inf')},
                                     {'imu': float('nan')})):
        invalid_body = {'instance_id': config['instance_id'],
                        'request_id': 'rates-invalid-' + str(index), 'rates_hz': invalid}
        assert http(socket_path, 'POST', '/telemetry-rates', invalid_body)[0] == 400, invalid_body
        assert http(socket_path, 'GET', '/telemetry-rates')[1] == stable
    after = http(socket_path, 'GET', '/status')[1]
    assert before['paused'] and after['paused'], (before, after)
    assert before['steps'] == after['steps'] == 0, (before, after)
    assert before['simulation_time_ns'] == after['simulation_time_ns'] == config['epoch_ns'], (before, after)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--xsim', required=True)
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix='xsim-headless-') as directory:
        root = Path(directory)
        config = {'instance_id': 'headless-native', 'epoch_ns': 1700000000000000000,
                  'paused': True, 'entities': [
                      {'name': kind, 'kind': kind} for kind in ('fs150', 'scout', 'mecanum')]}
        (root / 'world.json').write_text(json.dumps(config))
        socket_path = str(root / 'world.sock')
        with (root / 'server.log').open('w') as log:
            process = subprocess.Popen([args.xsim, '--config', str(root / 'world.json'),
                                        '--socket', socket_path], stdout=log, stderr=log)
            try:
                deadline = time.monotonic() + 5
                while not Path(socket_path).exists():
                    assert process.poll() is None, (root / 'server.log').read_text()
                    assert time.monotonic() < deadline, 'native socket not ready'
                    time.sleep(.005)
                telemetry_rates(socket_path, config)
                providers = [NativeProvider(socket_path, kind) for kind in ('fs150', 'scout', 'mecanum')]
                ids = [provider.entity_id for provider in providers]
                assert len(set(ids)) == 3 and min(ids) > 0, ids
                for provider in providers:
                    assert provider.read()['generation'] == 0 and not provider.read()['enabled']
                    started = provider.start(0)
                    assert started['success'] and started['enabled'] and started['generation'] == 1
                    # A stable predecessor generation does not reset an already active model.
                    repeated = provider.start(0)
                    assert repeated['success'] and repeated['generation'] == 1
                    stale = provider.stop(0)
                    assert not stale['success'] and stale['enabled'] and stale['generation'] == 1
                    stopped = provider.stop(1)
                    assert stopped['success'] and not stopped['enabled'] and stopped['generation'] == 1
                body = {'instance_id': 'headless-native', 'request_id': 'add-body', 'timeout_ms': 5000,
                        'entity': {'name': 'new_ground', 'kind': 'scout'}}
                code, admitted = http(socket_path, 'POST', '/entities', body)
                assert code == 202 and admitted['request_id'] == 'add-body', admitted
                added = receipt(socket_path, 'add-body')
                assert added['success'] and not added['enabled'] and added['generation'] == 0, added
                code, _ = http(socket_path, 'POST', '/entities', body)
                assert code == 200  # Same request ID reconciles the original Add.
                body['entity']['name'] = 'different'
                assert http(socket_path, 'POST', '/entities', body)[0] == 409
                remove = {'instance_id': 'headless-native', 'request_id': 'remove-body',
                          'generation': 0, 'timeout_ms': 5000}
                assert http(socket_path, 'DELETE', '/entities/' + str(added['entity_id']), remove)[0] == 202
                assert receipt(socket_path, 'remove-body')['success']
                foreign = {'instance_id': 'foreign', 'request_id': 'foreign-request',
                           'generation': 1, 'action': 'start', 'timeout_ms': 5000}
                assert http(socket_path, 'POST', '/entities/' + str(ids[0]) + '/provider', foreign)[0] == 409
                assert http(socket_path, 'GET', '/requests/unknown')[0] == 404
                status = http(socket_path, 'GET', '/status')[1]
                assert status['paused'] and status['simulation_time_ns'] == config['epoch_ns'], status
                assert status['model_step_ns'] == 2000000, status
                def mutation(path, request_id, extra=None):
                    body = {'instance_id': 'headless-native', 'request_id': request_id}
                    body.update(extra or {})
                    assert http(socket_path, 'POST', path, body)[0] == 202
                    return receipt(socket_path, request_id)
                manual = mutation('/step', 'manual-clock', {'steps': 3})
                assert manual['simulation_time_ns'] == config['epoch_ns'] + 6000000, manual
                mutation('/resume', 'run-clock')
                time.sleep(.05)
                mutation('/pause', 'pause-clock')
                running_status = http(socket_path, 'GET', '/status')[1]
                assert running_status['simulation_time_ns'] > manual['simulation_time_ns'], running_status
                assert 1000 <= running_status['last_dt_ns'] <= 10000000, running_status
                assert 0 < running_status['realtime_rtf'] <= 1.05, running_status
                assert running_status['scheduling_period_ns'] == 2000000, running_status
                frozen = running_status['simulation_time_ns']
                time.sleep(.01)
                assert http(socket_path, 'GET', '/status')[1]['simulation_time_ns'] == frozen
            finally:
                process.terminate()
                process.wait(timeout=10)
            assert process.returncode == 0, (root / 'server.log').read_text()
            assert not Path(socket_path).exists(), 'owned native socket survived clean exit'
    print('PASS: headless native telemetry rates, lifecycle, CAS, actual realtime clock, receipts, and socket retirement')


if __name__ == '__main__':
    main()
