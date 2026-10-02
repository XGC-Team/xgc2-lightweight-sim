#!/usr/bin/env python3
import hashlib
import json
from pathlib import Path

root = Path(__file__).resolve().parents[2]
lock = json.loads((root / '.xgc2/build-inputs/fs150.lock.json').read_text())
assert lock['schema'] == 'xgc2.trusted-build-input.v1'
assert lock['repository'] == 'XGC-Team/xgc2-fs150-sitl'
assert lock['source_sha'] == '65adfc04127a7331797fe2da4e3e236b7cdebefd'
digest = lock.pop('input_digest')
assert hashlib.sha256(json.dumps(lock, sort_keys=True, separators=(',', ':')).encode()).hexdigest() == digest
expected = {'scripts/generate_native_flight_model.py', 'models/fs150/iris.sdf', 'config/generated/fs150-sitl.params'}
assert set(lock['files']) == expected
for name, identity in lock['files'].items():
    data = (root / '.xgc2/build-inputs/fs150' / name).read_bytes()
    assert hashlib.sha256(data).hexdigest() == identity['sha256'], name
    assert len(data) == identity['size'], name
    blob = hashlib.sha1(('blob ' + str(len(data)) + '\0').encode() + data).hexdigest()
    assert blob == identity['git_blob'], name
print('PASS: exact owner FS150 source/blobs/SHA256 build inputs ' + digest)
