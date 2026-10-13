#!/usr/bin/env python3
"""Verify the saved checkpoint, including its outstanding merge finding."""
import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
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
review = json.loads(data['final_audit.json'])
assert review['final_acceptance'] == 'pending_merge_profile_fix'
assert len(review['confirmed_findings']) == 1
assert json.loads(data['summary.json'])['pass']
assert json.loads(data['r2/summary.json'])['numeric_exact']
assert b'70 passed' in data['python_installed.log']
assert b'FAIL: 1 failed checks' in data['merge_phase_probe/probe.log']
assert b'FAIL: 1 failed checks' in data['merge_phase_probe/default_probe.log']
if args.workspace:
    sources = json.loads(data['source_before.json'])
    for name, digest in sources.items():
        assert hashlib.sha256((args.workspace / name).read_bytes()).hexdigest() == digest, name
    print(f'PASS: {len(sources)} current source/configuration/test files match.')
print(f'PASS: {len(data)} saved files are intact; registered regression checks passed.')
print('Final acceptance remains pending: the merge-profile defect is reproduced and unfixed.')
