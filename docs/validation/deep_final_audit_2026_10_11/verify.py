#!/usr/bin/env python3
"""Offline integrity/equality checks of saved evidence; not a new ROS run."""
import argparse
import hashlib
import json
from pathlib import Path, PurePosixPath
import struct
import tarfile

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--workspace',type=Path,help='also compare reviewed code snapshot with this workspace')
args=parser.parse_args()
root=Path(__file__).resolve().parent
manifest=json.loads((root/'manifest.json').read_text())
archive=root/manifest['archive']['path']
assert archive.stat().st_size==manifest['archive']['bytes']
assert hashlib.sha256(archive.read_bytes()).hexdigest()==manifest['archive']['sha256']
expected={x['path']:x for x in manifest['files']}
data={}
with tarfile.open(archive,'r:gz') as tar:
 for member in tar:
  p=PurePosixPath(member.name)
  assert member.isfile() and not p.is_absolute() and '..' not in p.parts
  assert not {'reports','experiments'}&set(p.parts)
  assert member.name in expected and member.name not in data
  value=tar.extractfile(member).read()
  entry=expected[member.name]
  assert len(value)==entry['bytes'] and hashlib.sha256(value).hexdigest()==entry['sha256']
  data[member.name]=value
assert set(data)==set(expected)
def read(name):return json.loads(data[name])
s=read('final_validation.json');assert s==json.loads((root/'summary.json').read_text()) and s['pass']
t=read('tests/summary.json');assert t['pass'] and t['total_test_count']==54
assert data['runtime/result.json']==data['before_runtime/result.json']
assert hashlib.sha256(data['runtime/result.json']).hexdigest()==s['r2_owned_sha256']
assert read('runtime/summary.json')['pass'] and read('gui/summary.json')['pass']
assert read('inputs_before.json')['files']==read('inputs_after.json')['files']
assert len(read('inputs_before.json')['files'])==435
assert b'100% tests passed, 0 tests failed out of 45' in data['ubsan_tests.log']
assert b'runtime error:' not in data['sanitizer/ubsan_details.log']
assert b'[  PASSED  ] 41 tests.' in data['sanitizer/vehicle.log']
assert b'70 passed' in data['python_tests.log']
assert b'total checks=4413' in data['computation_probe/run_final.log']
c=read('chain_metrics.json');assert c['compared']==75 and c['exact_printed_match']==74
assert [k for k,v in c['results'].items() if not v['equal']]==['dynprobe.log']
assert read('fixed_yaw_comparison.json')['exact_printed_match']==1
# Expected bug fix: rejected 0.5-res map no longer poisons the accepted 0.7 pitch.
b=data['environment_lifecycle/before.bin'];a=data['environment_lifecycle/after.bin']
e=read('environment_lifecycle/comparison.json');offset=e['phases'][9]['offset']+71
assert e['pass_expected'] and e['all_query_values_exact']
assert struct.unpack_from('dd',b,offset)==(.5,.5)
assert struct.unpack_from('dd',a,offset)==(.7,.7)
assert b[:offset]==a[:offset] and b[offset+16:]==a[offset+16:]
for name in ['header_compile_commands.json','examples_compile_commands.json','runtime/build/compile_commands.json']:
 assert all('/ws/src/' not in item['command'] for item in read(name))
assert len(read('header_compile_commands.json'))==21
sources=read('source_snapshot_manifest.json')
for name,entry in sources.items():
 assert hashlib.sha256(data['source_snapshot/'+name]).hexdigest()==entry['sha256']
 if args.workspace:
  path=args.workspace/name
  assert not {'reports','experiments'}&set(path.parts)
  assert path.read_bytes()==data['source_snapshot/'+name],str(path)
if args.workspace:
 solver=read('solver_identity.json')
 path=args.workspace/'src/mmp_path_planning'/solver['file']
 assert hashlib.sha256(path.read_bytes()).hexdigest()==solver['sha256']
print(f"PASS: {len(data)} saved files; Release54/Python70/UBSan45/vehicle41; R2/ABORT equality and expected terrain correction verified.")
if args.workspace:print(f"PASS: {len(sources)} reviewed source snapshots and optimizer body match workspace.")
print('This verifies saved evidence; it does not rerun ROS, RViz, or the test binaries.')
