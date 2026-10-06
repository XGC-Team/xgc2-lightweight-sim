"""Native Unix HTTP transport for the private xsim contract fixtures."""
import http.client
import itertools
import json
import socket
import time


class UnixHTTP(http.client.HTTPConnection):
    def __init__(self, path):
        super().__init__('localhost', timeout=5)
        self.path = path

    def connect(self):
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.sock.settimeout(self.timeout)
        self.sock.connect(self.path)


_provider_requests = itertools.count(1)


def assert_retired_ros_surfaces_absent(system_state, bodies, ground_bodies):
    publishers, subscribers, services = system_state
    published = {name: set(nodes) for name, nodes in publishers}
    topics = set(published) | {name for name, _ in subscribers}
    service_names = {name for name, _ in services}
    retired_topics = {'/' + body + '/simulation/body_pose' for body in bodies}
    retired_topics |= {'/xgc/simulation/body/' + body + '/pose' for body in bodies}
    retired_topics |= {'/' + body + suffix for body in ground_bodies
                       for suffix in ('/simulation/pose', '/simulation/velocity', '/odom')}
    assert not topics & retired_topics, sorted(topics & retired_topics)
    retired_services = {'/' + body + suffix for body in bodies
                        for suffix in ('/simulation/reset', '/simulation/provider')}
    retired_services |= {'/xgc/lightweight/providers/' + body for body in bodies}
    assert not service_names & retired_services, sorted(service_names & retired_services)
    assert not any(name.endswith(('/simulation/reset', '/simulation/provider')) or
                   name.startswith('/xgc/lightweight/providers/') for name in service_names), service_names
    for body in bodies:
        for suffix in ('/pose', '/twist'):
            assert '/xsim' not in published.get('/' + body + suffix, set()), published
    return {'retired_topics_checked': sorted(retired_topics),
            'retired_services_checked': sorted(retired_services),
            'xsim_canonical_publishers': 0}


class NativeProvider:
    def __init__(self, socket_path, name):
        self.socket_path = socket_path
        self.name = name
        roster = self._get('/entities')
        self.instance_id = roster['instance_id']
        matches = [entity for entity in roster['entities'] if entity['name'] == name]
        assert len(matches) == 1, (name, roster)
        self.entity_id = matches[0]['id']
        assert type(self.entity_id) is int and self.entity_id > 0, matches[0]
        self._validate_entity(matches[0])

    def _http(self, method, path, body=None):
        connection = UnixHTTP(self.socket_path)
        try:
            connection.request(method, path, None if body is None else json.dumps(body),
                               {'Content-Type': 'application/json'})
            response = connection.getresponse()
            return response.status, json.loads(response.read())
        finally:
            connection.close()

    def _get(self, path):
        code, result = self._http('GET', path)
        assert code == 200, (code, result)
        return result

    def _validate_entity(self, entity):
        assert entity['name'] == self.name and entity['id'] == self.entity_id, entity
        assert type(entity['generation']) is int and entity['generation'] >= 0, entity
        assert type(entity['enabled']) is bool, entity
        assert entity['kind'] in ('fs150', 'scout', 'mecanum'), entity
        return entity

    def read(self):
        roster = self._get('/entities')
        assert roster['instance_id'] == self.instance_id, roster
        matches = [entity for entity in roster['entities'] if entity['id'] == self.entity_id]
        assert len(matches) == 1, (self.entity_id, roster)
        return self._validate_entity(matches[0])

    def start(self, generation):
        return self._mutate('start', generation)

    def stop(self, generation):
        return self._mutate('stop', generation)

    def _mutate(self, action, generation):
        assert type(generation) is int and generation >= 0, generation
        request_id = 'provider-' + str(next(_provider_requests))
        body = {'instance_id': self.instance_id, 'request_id': request_id,
                'generation': generation, 'action': action, 'timeout_ms': 5000}
        code, receipt = self._http('POST', '/entities/' + str(self.entity_id) + '/provider', body)
        assert code == 202 and receipt['instance_id'] == self.instance_id, (code, receipt)
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            receipt = self._get('/requests/' + request_id)
            assert receipt['instance_id'] == self.instance_id and receipt['request_id'] == request_id, receipt
            if receipt['phase'] in ('applied', 'cancelled', 'failed'):
                assert receipt['phase'] == 'applied' and receipt['result']['applied'], receipt
                result = receipt['result']
                assert result['entity_id'] == self.entity_id, result
                assert type(result['generation']) is int and result['generation'] >= 0, result
                assert type(result['enabled']) is bool and type(result['success']) is bool, result
                assert type(result['reason']) is int, result
                return result
            time.sleep(.005)
        raise AssertionError('timeout: provider ' + action + ' ' + self.name)
