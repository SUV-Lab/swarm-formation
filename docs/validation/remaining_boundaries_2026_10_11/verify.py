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
    assert summary['pass'] and summary['ctest'] == 53
    tests = data('tests/summary.json')
    assert tests['pass'] and tests['total_test_count'] == 53
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
    assert len(expected) == 75
    assert set(re.findall(r'^PASS (\S+)$', tally, re.M)) == expected
    assert not re.findall(r'^FAIL ', tally, re.M)
    assert all(f'tests/path_manager/chain_variants/{name}.log' in members for name in expected)
    baseline_tests = data('baseline/tests_summary.json')
    for package, suite in baseline_tests['suites'].items():
        assert set(suite['registered_names']) <= set(tests['suites'][package]['registered_names'])
    assert data('stage2_tests/summary.json')['pass']
    assert data('stage2_tests/summary.json')['total_test_count'] == 52
    assert data('solver_identity.json')['unchanged_vs_checkout']
    metric = data('chain_metrics_final.json')
    assert metric['compared'] == 73 and metric['exact_printed_match'] == 72
    assert [name for name, info in metric['results'].items() if not info['equal']] == ['dynprobe.log']
    fixed = data('fixed_yaw_comparison.json')
    assert fixed['compared'] == fixed['exact_printed_match'] == 1
    env = data('environment/summary.json')
    for label, names in [('numeric_masks', ('masks_before.bin','masks_installed.bin')),
                         ('numeric_lifecycle', ('lifecycle_before.bin','lifecycle_installed.bin'))]:
        before, after = [raw('environment/'+name) for name in names]
        assert before == after
        assert len(after) == env[label]['bytes']
        assert hashlib.sha256(after).hexdigest() == env[label]['sha256']

    cases = ET.fromstring(raw('all_tests.xml')).findall('.//testcase')
    assert len(cases) == 69
    assert not any(t.find(kind) is not None for t in cases for kind in ['failure', 'error', 'skipped'])
    imports = data('installed_test_imports.json')
    assert imports['before'] == imports['after']
    commands = data('examples_compile_commands.json')
    assert len(commands) == 4 and all('/ws/src/' not in c['command'] for c in commands)

    result = raw('runtime/result.json')
    assert result == raw('baseline/owned_r2.json')
    assert hashlib.sha256(result).hexdigest() == summary['owned_r2_sha256']
    runtime = data('runtime/summary.json')
    assert runtime['pass'] and runtime['numeric_exact'] and runtime['installed_only_compile']
    assert all(runtime['measurement_source_unchanged'].values())
    assert len(data('inputs_before.json')['files']) == 433
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
                  'ctest': 53, 'chain_variants': 75, 'python_tests': 69,
                  'owned_and_rviz_r2_exact': True, 'abort_recovery_exact': True,
                  'scope': 'saved evidence integrity/value audit; not a fresh runtime run'}, indent=2))
