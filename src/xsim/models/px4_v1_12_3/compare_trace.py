#!/usr/bin/env python3
"""Fail closed on missing steps, changed stimuli/history or differing outputs."""
import argparse
import json
import math
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument('reference', type=Path)
parser.add_argument('wrapper', type=Path)
parser.add_argument('--atol', type=float, default=1e-6)
parser.add_argument('--rtol', type=float, default=1e-6)
args = parser.parse_args()
assert args.atol >= 0 and args.rtol >= 0
reference = [json.loads(line) for line in args.reference.read_text().splitlines()]
wrapper = [json.loads(line) for line in args.wrapper.read_text().splitlines()]
assert len(reference) == len(wrapper) and reference, 'missing/additional history rows'
for expected, actual in zip(reference, wrapper):
    key = (expected['case'], expected['step'])
    assert key == (actual['case'], actual['step']), ('history order', key)
    for field in ('input', 'output'):
        assert len(expected[field]) == len(actual[field]), (key, field, 'field count')
        for index, (left, right) in enumerate(zip(expected[field], actual[field])):
            detail = (key, field, index, left, right)
            if isinstance(left, str) or isinstance(right, str):
                assert left == right and left in ('nan', 'inf', '-inf'), detail
            else:
                # The mixer status word is an exact original bitmask, not a
                # numeric measurement to be admitted by relative tolerance.
                exact = field == 'input' or (field == 'output' and expected['case'].startswith('mixer_')
                                             and index == len(expected[field]) - 1)
                if exact:
                    assert left == right, detail
                else:
                    assert math.isfinite(left) and math.isfinite(right), detail
                    assert math.isclose(left, right, abs_tol=args.atol, rel_tol=args.rtol), detail
print('Matched', len(reference), 'ordered original/wrapper history rows')
