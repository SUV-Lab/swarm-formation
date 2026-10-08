#!/usr/bin/env python3
"""Offline evidence checks using only Python's standard library.

This verifies saved evidence; it does not execute the C++ planner or ROS.
Use --source-root /path/to/MMP to compare the refreshed source allowlist.
"""
from pathlib import Path
from collections import Counter
import argparse
import gzip
import hashlib
import json
import math
import os
import re
import sys

ROOT = Path(__file__).resolve().parent
CHECKS = []


def check(condition, message):
    if not condition:
        raise ValueError(message)
    CHECKS.append(message)


def raw(name):
    path = ROOT / name
    if not path.is_file():
        path = ROOT / (name + '.gz')
    content = path.read_bytes()
    return gzip.decompress(content) if path.suffix == '.gz' else content


def obj(name):
    return json.loads(raw(name))


def text(name):
    return raw(name).decode()


def digest(content):
    return hashlib.sha256(content).hexdigest()


def exact(a, b):
    # Also preserve -0.0 and JSON numeric types; stricter than Python equality.
    return json.dumps(a, sort_keys=True) == json.dumps(b, sort_keys=True)


def numeric(traj):
    fields = ('order', 'duration', 'coef_x', 'coef_y', 'coef_z')
    result = {key: traj[key] for key in fields}
    n = len(result['duration'])
    check(n > 0 and all(math.isfinite(v) and v > 0 for v in result['duration']),
          'trajectory duration is positive and finite')
    for key in fields[2:]:
        check(len(result[key]) == n * (result['order'] + 1)
              and all(math.isfinite(v) for v in result[key]), 'coefficient array is complete: ' + key)
    return result


def last(capture, key):
    check(bool(capture[key]), 'capture contains ' + key)
    return capture[key][-1]['message']


def commands(command):
    return {k: v for k, v in command.items() if k not in ('mission_id', 'sequence')}


def check_hashes():
    manifest = obj('manifest.json')
    expected = {entry['path'] for entry in manifest['files']}
    actual = set()
    for root, directories, files in os.walk(ROOT):
        check(not set(directories) & {'reports', 'experiments'}, 'archive has no excluded directory')
        directories[:] = [d for d in directories if d not in ('__pycache__', '.git')]
        actual.update(str((Path(root)/name).relative_to(ROOT)) for name in files
                      if str((Path(root)/name).relative_to(ROOT)) != 'manifest.json')
    check(actual == expected, 'archive inventory has no missing or extra files')
    for entry in manifest['files']:
        path = Path(entry['path'])
        check(not path.is_absolute() and '..' not in path.parts
              and not set(path.parts) & {'reports', 'experiments'}, 'safe manifest path')
        payload = (ROOT/path).read_bytes()
        check(len(payload) == entry['bytes'] and digest(payload) == entry['sha256'],
              'archive hash: ' + str(path))
        decoded = gzip.decompress(payload) if entry['encoding'] == 'gzip' else payload
        check(len(decoded) == entry['source_bytes'] and digest(decoded) == entry['source_sha256'],
              'original byte hash: ' + str(path))
    final = obj('historical/ext_c/final_validation.json')
    for name, expected_hash in final['evidence_files'].items():
        check(digest(raw('historical/ext_c/'+name)) == expected_hash,
              'unchanged historical final index reference: '+name)
    check(final['pass'] is True, 'historical final index reports success')
    return manifest


def check_tests(prefix):
    summary = obj(prefix+'/after/tests/summary.json')
    total = 0
    for package, suite in summary['suites'].items():
        base = prefix+'/after/tests/'+package
        log = text(base+'/ctest.log')
        details = text(base+'/ctest_details.log')
        discovery = obj(base+'/ctest_discovery.json')
        names = sorted(test['name'] for test in discovery['tests'])
        count = len(re.findall(r'Test\s+#\d+:', log))
        check(names == suite['registered_names'], package+' registered test names match raw discovery')
        check(count == len(names) == suite['registered_test_count'] == suite['executed_test_count'],
              package+' execution counts match raw CTest output')
        check(suite['exit_code'] == 0 and suite['pass'] is True
              and f'100% tests passed, 0 tests failed out of {count}' in log
              and 'Permission denied' not in details, package+' raw CTest output passed')
        for name in names:
            chunk = text(base+'/'+name+'.log')
            check('Test Passed.' in chunk, package+'/'+name+' has a raw passed result')
        total += count
    check(total == summary['total_test_count'] == 37 and summary['pass'], '37 CTests passed')
    chain = prefix+'/after/tests/path_manager/chain_variants'
    tally = text(chain+'/tally.txt')
    names = re.findall(r'^PASS (\S+)$', tally, re.M)
    check(len(names) == len(set(names)) == 71 and not re.search(r'^FAIL ', tally, re.M)
          and tally.strip().endswith('DONE 71/71'), '71 chain cases passed in raw tally')
    for name in names:
        log = text(chain+'/'+name+'.log')
        check(not re.search(r'^\s*\[FAIL\]', log, re.M) and bool(log.strip()),
              'chain case raw evidence: '+name)
    risk = text(prefix+'/after/tests/path_manager/risk_scenarios.log')
    check('==== risk_scenarios_test: passed=154 failed=0 ====' in risk
          and len(re.findall(r'^\s*\[OK\]', risk, re.M)) == 154,
          '154 risk assertions passed in raw output')


def check_owned_results(prefix):
    baseline = obj('historical/pre_a/r2_consumer/result.json')
    mission_hash = digest(raw('historical/inputs/r2_ridge_crossing.yaml'))
    check(baseline['terrain']['mission_sha256'] == mission_hash, 'R2 input YAML matches historical request digest')
    for file in ('result.json', 'value_result.json', 'initial_environment_result.json'):
        result = obj(prefix+'/r2_consumer/'+file)
        check(result['serialized_after_planner_destruction'] and result['input_unchanged'],
              prefix+'/'+file+' records owned lifetime and unchanged input')
        if file == 'initial_environment_result.json':
            check(baseline['identity']['environment_generation'] == 6
                  and result['identity']['environment_generation'] == 4,
                  prefix+' initial environment generation is explicitly 6 to 4')
            result['identity']['environment_generation'] = 6
        check(exact(baseline, result), prefix+'/'+file+' exactly matches owned-double baseline except documented generation')
        check(result['final']['piece_count'] == 190
              and result['final']['duration'] == 521.0625247312521,
              prefix+' owned-double R2 has 190 pieces and recorded duration')


def check_gui(prefix):
    current = obj(prefix+'/gui/mission.json')
    prior = obj('historical/pre_a/gui/mission.json')
    original = obj('historical/original/baseline_mission.json')
    for field in ('trajectories', 'initial_trajectories'):
        current_numeric = numeric(last(current, field))
        for label, before in (('pre-A', prior), ('original', original)):
            check(exact(current_numeric, numeric(last(before, field))),
                  prefix+' '+field+' all wire coefficients/durations equal '+label)
        check(len(current_numeric['duration']) == 190
              and math.fsum(current_numeric['duration']) == 521.0625259429216,
              prefix+' wire R2 has 190 pieces and recorded duration')
        expected = obj(prefix+'/gui/comparison.json')['final' if field == 'trajectories' else 'initial']['after']['numeric_sha256']
        check(digest(json.dumps(current_numeric, sort_keys=True).encode()) == expected,
              prefix+' saved comparison numeric digest is independently reproduced')
    check(exact(commands(last(current,'commands')),commands(last(prior,'commands'))),
          prefix+' GUI mission input matches baseline except identity')
    for field in ('terrain_ready','obstacles','risk_zones'):
        check(exact([v['message'] for v in current[field]], [v['message'] for v in prior[field]]),
              prefix+' GUI environment messages match baseline: '+field)
    check(exact([p['pose'] for p in last(current,'front_end_paths')['poses']],
                [p['pose'] for p in last(prior,'front_end_paths')['poses']]),
          prefix+' frontend path exact')
    check(exact(last(current,'front_end_tubes')['points'],last(prior,'front_end_tubes')['points']),
          prefix+' frontend tube points exact')
    abort = obj(prefix+'/gui/abort.json')
    check(abort['pass'] is True and all(abort['checks'].values()) and len(abort['controls']) == 1,
          prefix+' ABORT observation checks pass with exactly one control')
    control = abort['controls'][0]['message']
    original_traj = last(current,'trajectories')
    recovered = last(abort,'live_trajectories')
    check(control['action'] == 1 and control['reason'] == 3
          and control['trajectory_id'] == original_traj['trajectory_id']
          and control['drone_id'] == original_traj['drone_id'], prefix+' ABORT action/reason/identity match')
    check(recovered['trajectory_id'] != original_traj['trajectory_id']
          and exact(numeric(recovered), numeric(original_traj)), prefix+' recovery has new identity and exact trajectory')
    check(exact(commands(abort['recovery_command']), commands(last(current,'commands'))),
          prefix+' recovery input differs only in identity')
    before = text('historical/pre_a/gui/planner.log').splitlines()
    after = text(prefix+'/gui/planner.log').splitlines()
    for name, tags, ordered, count in (
        ('evaluation', ('[CHAIN-REPORT]','[FINAL-EVAL]'),True,38),
        ('postprocessing', ('[Z-DENOISE]','[A*-SHORTCUT]','[CORNER-FILLET]'),True,2),
        ('cost_profile', ('[ALT-CAP]','[OCCLUSION-CAP]','[RIDE]'),False,10)):
        def select(lines): return [line.split('] ',2)[-1] for line in lines if any(tag in line for tag in tags)]
        a,b = select(before),select(after)
        check(len(a) == len(b) == count and (a == b if ordered else Counter(a) == Counter(b)),
              prefix+' raw '+name+' log counts/content match')


def check_consumers():
    p = 'historical/ext_c'
    baseline = 'historical/pre_a'
    for mode in ('direct','default'):
        a,b = obj(baseline+'/after/'+mode+'.json'),obj(p+'/after/'+mode+'.json')
        a.pop('times'); b.pop('times')
        check(exact(a,b), mode+' public consumer exact except execution times')
    for name, expected in (('model',48),('route',83),('mission_core',23)):
        log = text(p+'/consumer/'+name+'_run.log')
        check(len(re.findall(r'^\s*\[OK\]',log,re.M)) == expected
              and not re.search(r'^\s*\[FAIL\]',log,re.M), name+' public consumer assertion count')
    for name in ('mission_core_compile','value_headers'):
        check('/opt/ros/' not in text(p+'/consumer/'+name+'.log'), name+' trace has no ROS include path')
    check('path_manager' not in text(p+'/consumer/optimizer_link.txt'), 'optimizer link does not depend on path_manager')
    summary = obj(p+'/consumer/after_summary.json')
    for source, expected in summary['standalone_core_source_sha256'].items():
        check(digest(raw(p+'/consumer/standalone_'+Path(source).name)) == expected,
              'standalone facade source digest '+source)
    check(digest(raw(p+'/optimizer_consumer/main.cpp')) == summary['optimizer_consumer_source_sha256'],
          'optimizer consumer source matches test digest')
    installed = obj(baseline+'/installed_manifest.json')
    for entry in installed['files']:
        if entry['path'].endswith(('.yaml',)):
            check(digest(raw(baseline+'/config/'+Path(entry['path']).name)) == entry['sha256'],
                  'historical config snapshot matches installed manifest')


def check_source(source_root):
    manifest = obj('refresh/source_manifest_after.json')
    root = Path(source_root).resolve()
    entries = {entry['path']: entry for entry in manifest['files']}
    excluded = set(manifest['excluded_dir_names']) | {'reports','experiments'}
    present = set()
    for allowed in manifest['allowed_roots']:
        relative = Path(allowed)
        check(not relative.is_absolute() and '..' not in relative.parts
              and not set(relative.parts) & excluded, 'safe source allowlist path')
        start = root/relative
        if start.is_file():
            present.add(allowed)
        elif start.is_dir():
            for current, directories, files in os.walk(start):
                directories[:] = [d for d in directories if d not in excluded]
                present.update(str((Path(current)/name).relative_to(root)) for name in files)
    missing, added = sorted(set(entries)-present), sorted(present-set(entries))
    check(not missing and not added, 'source allowlist inventory unchanged; missing='+str(missing)+' added='+str(added))
    for name, entry in entries.items():
        path = root/name
        check(not set(path.parts) & {'reports','experiments'}, 'safe source path')
        content = path.read_bytes()
        check(len(content) == entry['bytes'] and digest(content) == entry['sha256'], 'refreshed source hash: '+name)
    return len(entries)


def check_refresh_identity():
    summary = obj('refresh/refresh_summary.json')
    for name, expected in summary['evidence_sha256'].items():
        check(digest(raw('refresh/'+name)) == expected, 'fresh summary evidence reference: '+name)
    before,after = obj('refresh/source_manifest_before.json'),obj('refresh/source_manifest_after.json')
    check(exact(before['files'],after['files']) and after['file_count'] == len(after['files']) == 147,
          'fresh source identities unchanged before and after validation')
    check(obj('refresh/build_summary.json')['exit_code'] == 0
          and obj('refresh/build_summary.json')['pass'] is True, 'fresh build recorded exit zero')
    check(re.search(r'Summary: 3 packages finished', text('refresh/build.log')) is not None,
          'fresh raw build output records three finished packages')
    installed = obj('refresh/installed_manifest_after.json')
    executable = next(entry['sha256'] for entry in installed['files']
                      if entry['path'].endswith('/path_manager/path_manager_node'))
    running_hashes = [line.split()[0] for line in text('refresh/gui/binary_hashes.txt').splitlines() if line.strip()]
    check(len(running_hashes) == 2 and set(running_hashes) == {executable},
          'fresh actual GUI executable identity equals post-build installed identity')
    check(summary['pass'] is True and summary['fresh_gui']['executed_node_sha256'] == executable,
          'fresh summary agrees with saved executed binary identity')
    check(not summary['historical_identity']['historical_source_identity_claimed'],
          'fresh source manifest is not attributed retroactively to historical tests')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-root', help='Optional MMP superproject root; verify refreshed source allowlist')
    args = parser.parse_args()
    manifest = check_hashes()
    check_tests('historical/ext_c')
    check_consumers()
    check_owned_results('historical/ext_c')
    check_gui('historical/ext_c')
    if manifest.get('refresh_included'):
        check_refresh_identity()
        check_tests('refresh')
        check_owned_results('refresh')
        check_gui('refresh')
    count = check_source(args.source_root) if args.source_root else None
    print(json.dumps({'pass': True, 'archive_files': len(manifest['files']),
                      'checks': len(CHECKS), 'refreshed_source_files_verified': count,
                      'mode': 'offline saved evidence verification; no build or ROS execution'}, indent=2))


if __name__ == '__main__':
    try:
        main()
    except (OSError, ValueError, KeyError, AssertionError) as error:
        print('FAIL: '+str(error), file=sys.stderr)
        raise SystemExit(1)
