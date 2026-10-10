#!/usr/bin/env python3
"""Offline evidence verification; standard Python only, no ROS or simulator."""
from pathlib import Path
import gzip, hashlib, json
root = Path(__file__).resolve().parent

def data(relative):
    p = root / relative
    if p.exists():
        return p.read_bytes()
    return gzip.decompress(Path(str(p) + '.gz').read_bytes())

def document(relative):
    return json.loads(data(relative))

index = document('artifact_index.json')
for row in index['files']:
    content = (root / row['path']).read_bytes()
    assert len(content) == row['bytes'], row['path']
    assert hashlib.sha256(content).hexdigest() == row['sha256'], row['path']

for suffix in ('events', 'late_full', 'late_clear'):
    assert data('focused/baseline/' + suffix + '_canonical.json') == data('focused/connected/' + suffix + '_canonical.json'), suffix
    assert document('focused/disconnected/' + suffix + '_canonical.json') == {}, suffix
    # Raw CDR + phase/topic/type index retained for every prepared payload.
    for scenario in ('baseline', 'connected'):
        records = data('focused/' + scenario + '/' + suffix + '/index.tsv').decode().splitlines()
        assert len(records) == {'events': 40, 'late_full': 11, 'late_clear': 11}[suffix]
        for record in records:
            phase, topic, message_type, filename = record.split('\t')
            assert data('focused/' + scenario + '/' + suffix + '/' + filename)

physical = data('focused/baseline/physical_state.txt')
for scenario in ('connected', 'disconnected'):
    assert physical == data('focused/' + scenario + '/physical_state.txt')
assert hashlib.sha256(physical).hexdigest() == '7df28eb4d4aa497dea6403f933f4561a40e94d890a1fb48553b84d1a0c4df70f'
for scenario in ('baseline', 'connected', 'disconnected'):
    checks = document('focused/' + scenario + '/canonical_summary.json')['checks']
    assert checks['all_expected_topics'] and checks['late_full_complete'] and checks['late_clear_complete']

reference = document('r2/reference_owned_result.json')
assert reference == document('r2/connected_result.json') == document('r2/disconnected_result.json')
assert reference['final']['piece_count'] == 190
for mode in ('connected', 'disconnected'):
    comparison = document('r2/' + mode + '_wire_comparison.json')
    assert comparison['pass'] and comparison['final']['numeric_exact'] and comparison['reference_trajectory']['numeric_exact']
assert document('r2/summary.json')['pass']
assert data('installed_before.sha256') == data('installed_after.sha256')
assert document('source_after_validation.json') == {'checked': 12, 'changed': [], 'pass': True}

# Bind source snapshot copies to the exact files frozen before the installed build.
for row in document('integration_manifest.json'):
    original = row['path']
    if '/src/path_manager/' in original:
        relative = original.split('/src/path_manager/', 1)[1]
    else:
        relative = 'docs/design/' + Path(original).name
    assert hashlib.sha256(data('source/current/' + relative)).hexdigest() == row['sha256'], relative

builder_log = data('focused/installed_builder_test.log').decode()
assert builder_log.count('[OK] ') == 28 and '[FAIL]' not in builder_log and 'failures=0' in builder_log
assert 'legacy zero path scale' in builder_log and 'legacy zero sphere mesh height' in builder_log
assert data('focused/installed_example.log').decode().strip() == 'route_points=2 tube_points=21 frame=map'
parent = document('runtime/parent_validation.json')
assert parent['pass'] and parent['ctest_count'] == 41 and parent['source_count'] == 12
suite = document('runtime/tests/summary.json')
assert suite['pass'] and suite['total_test_count'] == 41
assert suite['suites']['path_manager']['focused_tests']['planning_visualization_test']['ok_checks'] == 28
assert suite['suites']['path_manager']['scenario_evidence']['chain_pass_count'] == 71
assert suite['suites']['path_manager']['scenario_evidence']['risk_154_0']
assert document('runtime/gui/summary.json')['pass']
assert document('runtime/gui/comparison.json')['pass']
assert document('summary.json')['pass']
print(json.dumps({'pass': True, 'hashed_files': len(index['files']),
                  'focused_payloads_exact': 62, 'focused_topics': 11,
                  'physical_states_exact': 3, 'r2_modes_exact': 2,
                  'builder_checks': 28, 'ctest': '41/41', 'rviz_abort_recovery': True}, indent=2))
