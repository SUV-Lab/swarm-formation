#!/usr/bin/env python3
"""Verify saved merge-profile regression evidence and optional source identity."""
import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import re
import tarfile

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--workspace', type=Path)
args = parser.parse_args()
root = Path(__file__).resolve().parent
manifest = json.loads((root / 'manifest.json').read_text())
archive = root / manifest['archive']['path']
assert archive.stat().st_size == manifest['archive']['bytes']
assert hashlib.sha256(archive.read_bytes()).hexdigest() == manifest['archive']['sha256']
data = {}
with tarfile.open(archive, 'r:gz') as tar:
    for entry in tar:
        name = PurePosixPath(entry.name)
        assert entry.isfile() and not name.is_absolute() and '..' not in name.parts
        assert not {'reports', 'experiments'} & set(name.parts)
        assert entry.name in manifest['files'] and entry.name not in data
        value = tar.extractfile(entry).read()
        expected = manifest['files'][entry.name]
        assert len(value) == expected['bytes']
        assert hashlib.sha256(value).hexdigest() == expected['sha256']
        data[entry.name] = value
assert set(data) == set(manifest['files'])
result = json.loads(data['fix_validation.json'])
assert result['pass'] and result['ctest'] == 54
assert result['baseline_expected_failures'] == 4
assert result['postfix_contract_failed_checks'] == 0
assert b'FAIL: 4 failed checks' in data['baseline_probe.log']
assert b'PASS: 0 failed checks' in data['chain_profile_contract.log']
assert b'[FAIL]' not in data['chain_profile_contract.log']
for package, count in [('path_manager', 45), ('path_optimizer', 3), ('mmp_vehicle_dynamics', 6)]:
    assert len(json.loads(data[package + '_discovery.json'])['tests']) == count
    expected = f'100% tests passed, 0 tests failed out of {count}'.encode()
    assert expected in data['ctest_' + package + '.log']
assert b'DONE 75/75' in data['path_manager_details.log']
assert b'passed=154 failed=0' in data['path_manager_details.log']
assert json.loads(data['summary.json'])['pass']
assert json.loads(data['r2/summary.json'])['numeric_exact']
assert data['r2/result.json'] == data['r2/before_result.json']
assert json.loads(data['r2/result.json'])['final']['piece_count'] == 190
assert all('/ws/src/' not in c['command'] for c in json.loads(data['consumer_build/compile_commands.json']))
assert len(set(re.findall(rb'Built target (\w+)', data['headers_compile.log']))) == 21
assert result['source_identity']['optimizer_body_unchanged']
assert result['source_identity']['unexpected_changes'] == []
if args.workspace:
    sources = json.loads(data['current_sources.json'])
    for name, digest in sources.items():
        assert hashlib.sha256((args.workspace / name).read_bytes()).hexdigest() == digest, name
    print(f'PASS: {len(sources)} current source/configuration/test files match.')
print(f'PASS: {len(data)} saved files; regression failures 4 -> 0; CTest54/chain75/risk154; R2 exact.')
print('Saved evidence check only; no build or runtime test was launched.')
