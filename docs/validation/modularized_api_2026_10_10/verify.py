#!/usr/bin/env python3
"""Audit preserved hashes and exact owned R2 results; does not rerun ROS."""
from pathlib import Path
import hashlib,json,tarfile
root=Path(__file__).resolve().parent
manifest=json.loads((root/'manifest.json').read_text())
archive=root/manifest['archive']['path']
assert archive.stat().st_size==manifest['archive']['bytes']
assert hashlib.sha256(archive.read_bytes()).hexdigest()==manifest['archive']['sha256']
with tarfile.open(archive,'r:gz') as tar:
 members={m.name:m for m in tar.getmembers()}
 assert len(members)==len(manifest['files'])
 def raw(name):
  member=members[name]
  assert member.isfile() and not member.name.startswith('/') and '..' not in Path(member.name).parts
  return tar.extractfile(member).read()
 def data(name):return json.loads(raw(name))
 for f in manifest['files']:
  payload=raw(f['path'])
  assert len(payload)==f['bytes'],f['path']
  assert hashlib.sha256(payload).hexdigest()==f['sha256'],f['path']
 final=data('runtime/final_summary.json');assert final['pass']
 tests=data('runtime/tests/summary.json')
 assert tests['pass'] and tests['total_test_count']==41
 assert all(s['pass'] for s in tests['suites'].values())
 gui=data('runtime/gui/summary.json');assert gui['pass'] and gui['abort']['numeric_exact']
 consumer=data('consumer/summary.json')
 assert consumer['pass'] and consumer['frozen_inputs_unchanged']
 assert len(consumer['examples'])==4 and all(e['exit_code']==0 for e in consumer['examples'].values())
 assert consumer['example_compile_paths']['no_source_tree_include_or_source']
 assert consumer['r2']['pass'] and consumer['r2']['wire_pass']
 assert raw('consumer/r2/result.json')==raw('baseline/owned_result.json')
 assert data('consumer/inputs_before.json')==data('consumer/inputs_after.json')
 assert len(data('runtime/source_before.json')['files'])==183
print(json.dumps({'pass':True,'hashed_files':len(manifest['files']),'ctest':41,
 'external_cpp_examples':4,'owned_r2_byte_exact':True,'rviz_abort_recovery':True,
 'scope':'saved evidence audit and owned-result comparison, not a fresh runtime test'},indent=2))
