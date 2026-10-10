#!/usr/bin/env python3
"""Offline integrity and numerical comparison; does not replace live tests."""
import argparse
import hashlib
import json
from pathlib import Path
import tarfile

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--source-root', type=Path,
                    help='optional worktree at the stage 1/2 version')
args = parser.parse_args()
root = Path(__file__).resolve().parent
manifest = json.loads((root / 'manifest.json').read_text())
archive = root / 'evidence.tar.gz'
checks = 0


def check(condition, message):
    global checks
    if not condition:
        raise SystemExit('FAIL: ' + message)
    checks += 1


check(hashlib.sha256(archive.read_bytes()).hexdigest() == manifest['archive_sha256'],
      'archive hash')
with tarfile.open(archive, 'r:gz') as tar:
    expected = {entry['path']: entry for entry in manifest['files']}
    members = tar.getmembers()
    check(len(members) == len(expected), 'archive member count')
    payload = {}
    for member in members:
        name = member.name
        check(member.isfile() and name in expected and
              not Path(name).is_absolute() and '..' not in Path(name).parts,
              'regular, recorded relative member ' + name)
        data = tar.extractfile(member).read()
        entry = expected[name]
        check(len(data) == entry['bytes'] and
              hashlib.sha256(data).hexdigest() == entry['sha256'], name)
        payload[name] = data


def read(name):
    return json.loads(payload[name])


baseline = read('baseline/baseline_validation.json')
final = read('after/final_validation.json')
check(baseline['pass'] and baseline['live_tests']['ctest'] == 37, 'live baseline')
check(final['pass'] and final['ctest']['count'] == 39, 'live final tests')
check(final['ctest']['new_evaluator']['ok_checks'] == 67, 'evaluator checks')
check(final['ctest']['new_execution_adoption']['ok_checks'] == 62, 'adoption checks')
for prefix, summary in [('baseline/', baseline), ('after/', final)]:
    for name, info in summary['files'].items():
        check(hashlib.sha256(payload[prefix + name]).hexdigest() == info['sha256'],
              prefix + 'summary reference ' + name)
for filename in ['result.json', 'value_result.json', 'initial_environment_result.json']:
    old = read('baseline/r2_consumer/' + filename)
    new = read('after/r2_consumer/' + filename)
    for key in ['input_unchanged', 'serialized_after_planner_destruction']:
        check(new[key] is True, filename + ' ' + key)
    for key in ['status', 'final', 'reference', 'phases', 'evaluation', 'identity', 'terrain']:
        check(old[key] == new[key], filename + ' exact ' + key)

old_gui = read('baseline/gui/mission.json')
new_gui = read('after/gui/mission.json')
for channel in ['trajectories', 'initial_trajectories']:
    old = old_gui[channel][0]['message']
    new = new_gui[channel][0]['message']
    # Absolute receive/ROS clock stamps differ across runs. Execution IDs do not.
    for key in ['drone_id', 'trajectory_id', 'order', 'coef_x', 'coef_y', 'coef_z', 'duration']:
        check(old[key] == new[key], 'GUI ' + channel + ' ' + key)
abort = read('after/gui/abort.json')
check(abort['pass'] and all(abort['checks'].values()), 'actual ABORT/recovery')
check(abort['original_trajectory_id'] == 1 and abort['recovered_trajectory_id'] == 2,
      'ABORT/recovery identities')
check(abort['recovery_numeric_exact_vs_original'], 'recovery coefficients')
for name in ['comparison', 'frontend_comparison', 'evaluation_comparison',
             'postprocessing_comparison', 'cost_profile_comparison']:
    check(read('after/gui/' + name + '.json')['pass'], 'GUI ' + name)

oracle = read('evaluator/summary.json')
check(oracle['passed'] and oracle['unit_checks_passed'] == 67 and
      oracle['baseline_differential_fixtures'] == 152 and
      oracle['all_verdict_fields_and_log_lines_exact'], 'independent original evaluator oracle')
check(read('after/evaluator_consumer/summary.json')['pass'], 'installed evaluator/docs consumer')
source = read('after/source_manifest_after.json')
check(source['files'] == read('after/source_manifest_before.json')['files'],
      'source unchanged during build and tests')
for entry in source['files']:
    check(hashlib.sha256(payload['after/source/' + entry['path']]).hexdigest() == entry['sha256'],
          'tested source snapshot ' + entry['path'])
    if args.source_root:
        path = args.source_root / entry['path']
        check(path.is_file() and hashlib.sha256(path.read_bytes()).hexdigest() == entry['sha256'],
              'worktree at stage 1/2 ' + entry['path'])
print('PASS:', checks, 'offline integrity/numerical checks; live execution not repeated')
