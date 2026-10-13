#!/usr/bin/env python3
"""Verify preserved artifacts; optionally compare current source. Does not build/run ROS."""
import argparse, hashlib, json, tarfile
from pathlib import Path
p=argparse.ArgumentParser(); p.add_argument('--workspace',type=Path); args=p.parse_args()
root=Path(__file__).resolve().parent
manifest=json.loads((root/'manifest.json').read_text())
sha=lambda b:hashlib.sha256(b).hexdigest()
archive=root/'evidence.tar.gz'
assert sha(archive.read_bytes())==manifest['archive']['sha256']
with tarfile.open(archive) as tar:
    members={m.name:m for m in tar.getmembers() if m.isfile()}
    assert set(members)==set(manifest['files'])
    for name,record in manifest['files'].items():
        path=Path(name)
        assert not path.is_absolute() and '..' not in path.parts
        assert not {'reports','experiments'} & set(path.parts)
        data=tar.extractfile(members[name]).read()
        assert len(data)==record['bytes'] and sha(data)==record['sha256'],name
    summary=json.loads(tar.extractfile('summary.json').read())
    assert summary['pass']
    if args.workspace:
        snap=json.loads(tar.extractfile('source_frozen.json').read())
        for entry in snap['files']:
            rel=Path(entry['path'])
            assert not {'reports','experiments'}&set(rel.parts)
            assert sha((args.workspace/rel).read_bytes())==entry['sha256'],str(rel)
print('PASS: archive hashes, validation summary'+(', current source fingerprints' if args.workspace else ''))
