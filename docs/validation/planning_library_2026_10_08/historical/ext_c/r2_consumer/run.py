from pathlib import Path
import datetime, json, os, signal, subprocess, time

root = Path('/tmp/mmp_modularization_ext_c/r2_consumer')
assert os.environ.get('ROS_DOMAIN_ID') == '84'
summary = {'started_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
           'ros_domain_id': 84, 'checks': {}, 'modes': {}}
def run(name, cmd, timeout=300, env=None):
    started = time.monotonic()
    with (root / (name + '.log')).open('w') as out:
        try:
            rc = subprocess.run(cmd, cwd=root, stdout=out, stderr=subprocess.STDOUT,
                                timeout=timeout, env=env).returncode
        except subprocess.TimeoutExpired:
            rc = 124
    summary['checks'][name] = {'command': cmd, 'exit_code': rc, 'elapsed_s': time.monotonic() - started}
    print(name, rc, flush=True)
    return rc == 0

def stop_loader(terrain):
    if terrain is not None and terrain.poll() is None:
        os.killpg(terrain.pid, signal.SIGINT)
        try:
            terrain.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(terrain.pid, signal.SIGTERM)
            terrain.wait(timeout=10)

try:
    if not run('configure', ['cmake', '-S', str(root), '-B', str(root / 'build'), '-DCMAKE_BUILD_TYPE=Release']):
        raise RuntimeError('configure failed')
    if not run('build', ['cmake', '--build', str(root / 'build'), '-j2']):
        raise RuntimeError('build failed')
    baseline_path = Path('/tmp/mmp_modularization_ext_b/r2_consumer/result.json')
    reference = json.loads(baseline_path.read_text())
    for mode in ('bound_node', 'value_input', 'initial_environment'):
        terrain = None
        result = root / ({'bound_node':'result.json', 'value_input':'value_result.json',
                         'initial_environment':'initial_environment_result.json'}[mode])
        comparison = root / (mode + '_comparison.json')
        environment = os.environ.copy()
        environment.pop('MMP_VALUE_INPUT', None)
        if mode != 'bound_node':
            environment['MMP_VALUE_INPUT'] = mode
        try:
            with (root / (mode + '_terrain_loader.log')).open('w') as log:
                terrain = subprocess.Popen(['ros2', 'run', 'mmp_terrain', 'terrain_publisher', '--ros-args', '-p', 'world:=none'],
                                           cwd=root, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                summary['modes'][mode] = {'own_loader_pid': terrain.pid}
                if not run(mode + '_run', [str(root / 'build/r2_external_consumer'),
                        '/ws/install/path_manager/share/path_manager/config/optimizer_params.yaml',
                        '/ws/install/path_manager/share/path_manager/config/drone_hardware.yaml',
                        '/ws/src/mmp_terrain/data/scenarios/missions/r2_ridge_crossing.yaml', str(result)], 900, environment):
                    raise RuntimeError(mode + ' R2 library solve failed')
                if not run(mode + '_compare', ['python3', str(root / 'compare.py'),
                        '/tmp/mmp_modularization_stage9/after/mission.json', str(result), str(comparison)]):
                    raise RuntimeError(mode + ' R2 wire comparison failed')
                current = json.loads(result.read_text())
                exact = {key: reference[key] == current.get(key) for key in reference}
                check = {'reference': str(baseline_path), 'result': str(result),
                         'field_exact': exact, 'pass': all(exact.values())}
                if mode == 'initial_environment':
                    # Omitting the initial empty-world clear skips one terrain
                    # and one effective-zone generation increment. This is an
                    # intentional input-history difference, never numerical drift.
                    baseline_generation = reference['identity']['environment_generation']
                    actual_generation = current['identity']['environment_generation']
                    other_identity = {key: reference['identity'][key] == current['identity'].get(key)
                                      for key in reference['identity'] if key != 'environment_generation'}
                    contract = {'baseline_generation':baseline_generation,
                                'actual_generation':actual_generation,
                                'expected_delta':-2,
                                'actual_delta':actual_generation-baseline_generation,
                                'reason':'Constructor receives the actual terrain directly, so no prior empty-world clear increments terrain and effective-zone generations.',
                                'other_identity_fields_exact':other_identity}
                    contract['pass'] = contract['actual_delta'] == contract['expected_delta'] and all(other_identity.values())
                    check['initial_environment_contract'] = contract
                    check['pass'] = contract['pass'] and all(value for key,value in exact.items() if key != 'identity')
                (root / (mode + '_owned_double_comparison.json')).write_text(json.dumps(check, indent=2) + '\n')
                summary['modes'][mode]['owned_double_exact'] = all(exact.values())
                summary['modes'][mode]['pass'] = check['pass']
                if mode == 'initial_environment':
                    summary['modes'][mode]['initial_environment_contract'] = contract
                if not check['pass']:
                    raise RuntimeError(mode + ' R2 owned-double comparison differs')
                if mode != 'bound_node':
                    log_text = (root / (mode + '_run.log')).read_text()
                    count = next(int(line.split()[-1]) for line in log_text.splitlines() if line.startswith('OWNED_SETTING_COUNT '))
                    summary['modes'][mode]['owned_setting_count'] = count
                    if count < 10:
                        raise RuntimeError('value-input test did not pass actual typed settings')
                    if 'PLANNING_NODE_CREATED_BY_BACKEND true' not in log_text:
                        raise RuntimeError('value-input test did not create its planning node inside the explicit backend')
                    if mode == 'initial_environment' and 'OWNED_TERRAIN_AT_CONSTRUCTION true' not in log_text:
                        raise RuntimeError('initial-environment test did not pass terrain to the constructor')
        finally:
            stop_loader(terrain)
    summary['owned_double_exact'] = all(summary['modes'][mode]['owned_double_exact'] for mode in ('bound_node', 'value_input'))
    summary['initial_environment_pass'] = summary['modes']['initial_environment']['pass']
except Exception as error:
    summary['error'] = str(error)
finally:
    summary['finished_utc'] = datetime.datetime.now(datetime.timezone.utc).isoformat()
    summary['pass'] = 'error' not in summary and summary.get('owned_double_exact', False) and summary.get('initial_environment_pass', False) and all(
        summary['checks'].get(name, {}).get('exit_code') == 0 for name in (
            'configure', 'build', 'bound_node_run', 'bound_node_compare', 'value_input_run', 'value_input_compare',
            'initial_environment_run', 'initial_environment_compare'))
    (root / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(summary, indent=2), flush=True)
raise SystemExit(0 if summary['pass'] else 1)
