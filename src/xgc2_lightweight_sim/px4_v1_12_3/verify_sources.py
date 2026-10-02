#!/usr/bin/env python3
"""Verify the immutable per-file extraction; --remote also checks official raw."""
import argparse
import hashlib
import json
from pathlib import Path
import urllib.request

root = Path(__file__).resolve().parent
parser = argparse.ArgumentParser()
parser.add_argument('--remote', action='store_true')
args = parser.parse_args()
manifest = json.loads((root / 'source-manifest.json').read_text())
assert manifest['commit'] == '2e8918da66af37922ededee1cc2d2efffec4cfb2'
for entry in manifest['files']:
    data = (root / entry['path']).read_bytes()
    assert hashlib.sha256(data).hexdigest() == entry['sha256'], entry['path']
    assert len(data) == entry['bytes'], entry['path']
    if args.remote:
        request = urllib.request.Request(entry['url'], headers={'User-Agent': 'xgc-px4-source-verification'})
        remote = urllib.request.urlopen(request, timeout=30).read()
        assert remote == data, entry['url']
for entry in manifest['local_generated']:
    data = (root / entry['path']).read_bytes()
    assert hashlib.sha256(data).hexdigest() == entry['sha256'], entry['path']
print('Verified', len(manifest['files']), 'immutable upstream files and',
      len(manifest['local_generated']), 'explicit generated/shim files')
