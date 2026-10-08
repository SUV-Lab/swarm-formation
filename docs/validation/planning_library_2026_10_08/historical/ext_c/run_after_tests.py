#!/usr/bin/env python3
"""Post-build regression only. Run as user with sourced install, domain81."""
from pathlib import Path
import datetime, json, os, re, shutil, subprocess, time

root = Path('/tmp/mmp_modularization_ext_c')
out = root / 'after/tests'
assert os.environ.get('ROS_DOMAIN_ID') == '81'
out.mkdir(parents=True, exist_ok=True)
summary = {'started_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
           'ros_domain_id': 81, 'suites': {}, 'pass': False}
prior = json.loads(Path('/tmp/mmp_modularization_ext_b/after/tests/summary.json').read_text())
required = {package: set(info['registered_names']) for package, info in prior['suites'].items()}
for package in required:
    build = Path('/ws/build') / package
    package_out = out / package
    package_out.mkdir(exist_ok=True)
    discovery = subprocess.run(['ctest', '--show-only=json-v1'], cwd=build,
                               capture_output=True, text=True, timeout=30)
    (package_out / 'ctest_discovery.json').write_text(discovery.stdout)
    (package_out / 'ctest_discovery.stderr.log').write_text(discovery.stderr)
    if discovery.returncode:
        raise RuntimeError(f'{package} CTest discovery failed: {discovery.stderr}')
    names = {entry['name'] for entry in json.loads(discovery.stdout)['tests']}
    if not required[package].issubset(names):
        raise RuntimeError(f'{package} lost registered tests: {required[package] - names}')
    started = time.time()
    begin = time.monotonic()
    with (package_out / 'ctest.log').open('w') as log:
        try:
            done = subprocess.run(['ctest', '--output-on-failure'], cwd=build,
                                  stdout=log, stderr=subprocess.STDOUT, timeout=1500)
            rc = done.returncode
        except subprocess.TimeoutExpired:
            rc = 124
    source_log = build / 'Testing/Temporary/LastTest.log'
    shutil.copy2(source_log, package_out / 'ctest_details.log')
    results = build / 'test_results'
    if results.exists():
        shutil.copytree(results, package_out / 'test_results', dirs_exist_ok=True,
                        ignore=shutil.ignore_patterns('reports', 'experiments'))
    raw = (package_out / 'ctest_details.log').read_text()
    short = (package_out / 'ctest.log').read_text()
    focused = {}
    for chunk in re.split(r'(?m)^(?=\d+/\d+ Testing:)', raw):
        first = chunk.splitlines()[0] if chunk.splitlines() else ''
        match = re.match(r'^\d+/\d+ Testing: (\S+)$', first)
        if not match:
            continue
        name = match.group(1)
        if '/' in name or name in {'reports', 'experiments'}:
            raise RuntimeError('Unsafe CTest name for artifact path')
        (package_out / (name + '.log')).write_text(chunk)
        focused[name] = {'ok_checks': sum(line.lstrip().startswith('[OK]') for line in chunk.splitlines()),
                         'failed_checks': sum(line.lstrip().startswith('[FAIL]') for line in chunk.splitlines()),
                         'pass': 'Test Passed.' in chunk}
    suite = {'exit_code': rc, 'registered_test_count': len(names), 'registered_names': sorted(names),
             'executed_test_count': len(re.findall(r'Test\s+#\d+:', short)),
             'elapsed_s': time.monotonic() - begin,
             'last_log_written_after_start': source_log.stat().st_mtime >= started,
             'permission_denied_count': raw.count('Permission denied'),
             'focused_tests': focused,
             'summary_lines': [line for line in short.splitlines()
                               if re.search(r'Test\s+#\d+:', line) or 'tests passed' in line or 'Total Test time' in line]}
    suite['pass'] = rc == 0 and suite['executed_test_count'] == len(names) and bool(names) \
        and suite['last_log_written_after_start'] and suite['permission_denied_count'] == 0 \
        and len(focused) == len(names) and all(info['pass'] for info in focused.values())
    if package == 'path_manager':
        chain_dir = build / 'chain_variants'
        shutil.copytree(chain_dir, package_out / 'chain_variants', dirs_exist_ok=True,
                        ignore=shutil.ignore_patterns('reports', 'experiments'))
        tally = (chain_dir / 'tally.txt').read_text()
        logs = list(chain_dir.glob('*.log'))
        evidence = {'chain_pass_count': len(re.findall(r'^PASS ', tally, re.M)),
                    'chain_failure_count': len(re.findall(r'^FAIL ', tally, re.M)),
                    'chain_log_count': len(logs),
                    'all_chain_logs_written_after_start': all(p.stat().st_mtime >= started for p in logs),
                    'chain_tally_last_line': tally.strip().splitlines()[-1],
                    'risk_154_0': '==== risk_scenarios_test: passed=154 failed=0 ====' in raw}
        suite['scenario_evidence'] = evidence
        suite['pass'] = suite['pass'] and evidence['chain_pass_count'] == 71 \
            and evidence['chain_failure_count'] == 0 and evidence['chain_log_count'] == 71 \
            and evidence['all_chain_logs_written_after_start'] and evidence['risk_154_0']
    summary['suites'][package] = suite
    (out / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(package, 'PASS' if suite['pass'] else 'FAIL', suite['executed_test_count'], 'tests', flush=True)
summary['total_test_count'] = sum(s['executed_test_count'] for s in summary['suites'].values())
summary['pass'] = all(s['pass'] for s in summary['suites'].values())
summary['finished_utc'] = datetime.datetime.now(datetime.timezone.utc).isoformat()
(out / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
print(json.dumps(summary, indent=2), flush=True)
raise SystemExit(0 if summary['pass'] else 1)
