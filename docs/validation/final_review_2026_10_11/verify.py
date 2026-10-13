#!/usr/bin/env python3
"""Audit saved evidence. This does not rebuild or rerun ROS/RViz/tests."""
from pathlib import Path
import hashlib
import json
import re
import tarfile
import xml.etree.ElementTree as ET

root = Path(__file__).resolve().parent
manifest = json.loads((root / 'manifest.json').read_text())
archive = root / manifest['archive']['path']
assert archive.stat().st_size == manifest['archive']['bytes']
assert hashlib.sha256(archive.read_bytes()).hexdigest() == manifest['archive']['sha256']
with tarfile.open(archive, 'r:gz') as tar:
    entries = tar.getmembers()
    members = {entry.name: entry for entry in entries}
    assert len(entries) == len(members) == len(manifest['files'])

    def raw(name):
        entry = members[name]
        assert entry.isfile() and not Path(name).is_absolute()
        assert not {'..', 'reports', 'experiments'}.intersection(Path(name).parts)
        return tar.extractfile(entry).read()

    def data(name):
        return json.loads(raw(name))

    for item in manifest['files']:
        contents = raw(item['path'])
        assert len(contents) == item['bytes'], item['path']
        assert hashlib.sha256(contents).hexdigest() == item['sha256'], item['path']
    summary = data('final_validation.json')
    assert summary == json.loads((root / 'summary.json').read_text())
    assert summary['pass'] and summary['ctest'] == 48
    tests = data('tests/summary.json')
    assert tests['pass'] and tests['total_test_count'] == 48
    for suite in tests['suites'].values():
        assert suite['pass'] and suite['exit_code'] == 0
        assert suite['executed_test_count'] == suite['registered_test_count']
        assert suite['registered_test_count'] == len(suite['focused_tests'])
        assert all(t['pass'] for t in suite['focused_tests'].values())
        assert suite['last_log_written_after_start'] and suite['permission_denied_count'] == 0
    pm = tests['suites']['path_manager']
    assert pm['scenario_evidence']['risk_154_0']
    source = raw('source_snapshot/src/path_manager/test/chain_experiment_test.cpp').decode()
    expected = set(re.findall(r'if \(v == "([a-z0-9_]+)"\)', source))
    tally = raw('tests/path_manager/chain_variants/tally.txt').decode()
    assert len(expected) == 73
    assert set(re.findall(r'^PASS (\S+)$', tally, re.M)) == expected
    assert not re.findall(r'^FAIL ', tally, re.M)
    assert all(f'tests/path_manager/chain_variants/{name}.log' in members for name in expected)
    initial = data('tests/summary_initial_71_expectation.json')
    assert not initial['pass']
    assert initial['suites']['path_manager']['exit_code'] == 0
    assert initial['suites']['path_manager']['scenario_evidence']['chain_pass_count'] == 73

    cases = ET.fromstring(raw('all_tests.xml')).findall('.//testcase')
    assert len(cases) == 69
    assert not any(t.find(kind) is not None for t in cases for kind in ['failure', 'error', 'skipped'])
    imports = data('installed_test_imports.json')
    assert imports['before'] == imports['after']
    commands = data('examples_compile_commands.json')
    assert len(commands) == 4 and all('/ws/src/' not in c['command'] for c in commands)

    case_names = ['3_segdiff_tailacc', '3_segdiff_tailacc_route',
                  '2_tailcap', '2_tailcap_route', '3_prefixfallback']
    for case in case_names:
        before = raw(f'chain_reproductions/before/{case}.log').decode()
        after = raw(f'chain_reproductions/after/{case}.log').decode()
        assert ('[FAIL]' in before) == (case != '2_tailcap_route'), case
        assert '[FAIL]' not in after and 'PASS: 0 failed check(s)' in after, case
    override_before = raw('repro_override.log').decode()
    for mode in ['direct', 'chain']:
        assert f'[FAIL] {mode} successfully retries after refusals' in override_before
        assert f'[OK] {mode} refused overrides preserve the next normal solve exactly' in raw('tests/path_manager/mission_planner_test.log').decode()

    result = raw('runtime/result.json')
    assert result == raw('baseline/owned_r2.json')
    assert hashlib.sha256(result).hexdigest() == summary['owned_r2_sha256']
    runtime = data('runtime/summary.json')
    assert runtime['pass'] and runtime['numeric_exact'] and runtime['installed_only_compile']
    assert all(runtime['measurement_source_unchanged'].values())
    assert len(data('inputs_before.json')['files']) == 421
    assert data('inputs_before.json')['files'] == data('inputs_after.json')['files']

    baseline = data('baseline/gui/mission.json')
    current = data('gui/mission.json')
    numeric_fields = ['order', 'coef_x', 'coef_y', 'coef_z', 'duration']
    for topic in ['trajectories', 'initial_trajectories']:
        assert len(baseline[topic]) == len(current[topic]) == 1
        for field in numeric_fields:
            assert baseline[topic][0]['message'][field] == current[topic][0]['message'][field]
    abort = data('gui/abort.json')
    assert abort['pass'] and abort['recovery_numeric_exact_vs_original']
    assert all(abort['checks'].values())
    recovered = abort['live_trajectories'][-1]['message']
    original = current['trajectories'][0]['message']
    assert recovered['trajectory_id'] != original['trajectory_id']
    assert all(recovered[k] == original[k] for k in numeric_fields)
    assert data('gui/summary.json')['pass']
    assert data('gui/frontend_comparison.json')['pass']
    hashes = [line.split()[0] for line in raw('gui/binary_hashes.txt').decode().splitlines()]
    assert len(hashes) == 2 and len(set(hashes)) == 1

print(json.dumps({'pass': True, 'hashed_files': len(manifest['files']),
                  'ctest': 48, 'chain_variants': 73, 'python_tests': 69,
                  'owned_and_rviz_r2_exact': True, 'abort_recovery_exact': True,
                  'scope': 'saved evidence integrity/value audit; not a fresh runtime run'}, indent=2))
