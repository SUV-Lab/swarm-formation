#!/usr/bin/env python3
"""Check preserved evidence/values; this does not rerun ROS or benchmarks."""
from pathlib import Path
import hashlib,json,tarfile
root=Path(__file__).resolve().parent
manifest=json.loads((root/'manifest.json').read_text())
archive=root/manifest['archive']['path']
assert archive.stat().st_size==manifest['archive']['bytes']
assert hashlib.sha256(archive.read_bytes()).hexdigest()==manifest['archive']['sha256']
with tarfile.open(archive,'r:gz') as tar:
    entries=tar.getmembers()
    members={entry.name:entry for entry in entries}
    assert len(members)==len(entries)==len(manifest['files'])
    def raw(name):
        entry=members[name]
        assert entry.isfile() and not entry.name.startswith('/')
        assert not {'..','reports','experiments'}.intersection(Path(name).parts)
        return tar.extractfile(entry).read()
    def data(name): return json.loads(raw(name))
    for item in manifest['files']:
        content=raw(item['path'])
        assert len(content)==item['bytes'],item['path']
        assert hashlib.sha256(content).hexdigest()==item['sha256'],item['path']
    final=data('final_validation.json')
    assert final==json.loads((root/'summary.json').read_text())
    assert final['pass'] and final['ctest']==43
    tests=data('after/tests/summary.json')
    assert tests['pass'] and tests['total_test_count']==43
    scenarios=tests['suites']['path_manager']['scenario_evidence']
    assert scenarios['chain_pass_count']==71 and scenarios['risk_154_0']
    for suite in tests['suites'].values():
        assert suite['pass'] and suite['executed_test_count']==suite['registered_test_count']
    assert data('checkpoint1/tests/summary.json')['pass']
    assert data('checkpoint1/tests/summary.json')['total_test_count']==42
    reference=raw('baseline/run_00/result.json')
    assert hashlib.sha256(reference).hexdigest()=='33b40dc818378be2343349bd31bff20c309bbb7aa2725d1f00e76b88ce1ebfe5'
    for phase in ['baseline','after']:
        for index in range(6):
            assert raw(f'{phase}/run_{index:02d}/result.json')==reference
        gui=data(f'{phase}/gui/summary.json')
        assert gui['pass'] and gui['abort']['numeric_exact']
    runtime=data('after/runtime_final.json')
    assert runtime['pass'] and runtime['R2_results_exact']==6
    assert runtime['installed_only_compile'] and runtime['measurement_source_exact']
    before=data('after/inputs_before.json')['files']
    assert len(before)==421
    assert before==data('after/inputs_after_benchmark.json')['files']
    assert before==data('after/inputs_after_gui.json')['files']
    terrain_before=raw('environment/baseline_fixture/result.bin')
    terrain_after=raw('environment/after_fixture/result.bin')
    assert terrain_before==terrain_after and len(terrain_after)==12053232
    terrain=data('environment/fixture_comparison.json')
    assert terrain['pass'] and terrain['queries']==203520
    assert hashlib.sha256(terrain_after).hexdigest()==terrain['after_sha256']
    fixture=data('environment/fixture_timing/summary.json')
    assert fixture['all_results_exact'] and fixture['binaries_unchanged']
    assert data('after/solver_identity.json')['same_optimizer_outside_parameter_reader']
    chain=data('after/chain_metric_comparison.json')
    assert chain['pass'] and chain['variants']==71 and chain['total_metrics']==5160
print(json.dumps({'pass':True,'hashed_files':len(manifest['files']),'ctest':43,
    'owned_r2_exact_runs':12,'terrain_queries_exact':203520,
    'rviz_abort_recovery':True,'scope':'saved evidence and exact-value audit; not a fresh runtime test'},indent=2))
