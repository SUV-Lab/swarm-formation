#!/usr/bin/env python3
"""Compare live ROS captures. Numeric units are those of published PolyTraj.
Ignores command mission_id/sequence and trajectory identity/start time only.
ROS receipt/wall times are not compared. Coefficients are highest power first.
"""
import argparse
import hashlib
import json
import sys
from pathlib import Path
import numpy as np


def last(capture, key):
    if not capture[key]:
        raise ValueError('Missing ' + key)
    return capture[key][-1]['message']


def without_keys(value, ignored):
    if isinstance(value, dict):
        return {k: without_keys(v, ignored) for k, v in value.items() if k not in ignored}
    if isinstance(value, list):
        return [without_keys(v, ignored) for v in value]
    return value


def samples(traj, times, derivative):
    order = traj['order']
    duration = np.asarray(traj['duration'], dtype=float)
    ends = np.cumsum(duration)
    pieces = np.clip(np.searchsorted(ends, times, side='right'), 0, len(duration)-1)
    local = times - np.concatenate(([0.0], ends[:-1]))[pieces]
    result = np.empty((len(times), 3))
    for axis, name in enumerate(('coef_x', 'coef_y', 'coef_z')):
        coeff = np.asarray(traj[name], dtype=float).reshape((-1, order+1))
        for _ in range(derivative):
            coeff = coeff[:, :-1] * np.arange(coeff.shape[1]-1, 0, -1)
        values = np.zeros(len(times))
        for column in range(coeff.shape[1]):
            values = values * local + coeff[pieces, column]
        result[:, axis] = values
    return result


def trajectory_summary(traj):
    duration = np.asarray(traj['duration'], dtype=float)
    if len(duration) == 0 or not np.all(np.isfinite(duration)) or np.any(duration <= 0):
        raise ValueError('Invalid trajectory durations')
    for key in ('coef_x', 'coef_y', 'coef_z'):
        if len(traj[key]) != len(duration)*(traj['order']+1) or not np.all(np.isfinite(traj[key])):
            raise ValueError('Invalid coefficients: ' + key)
    total = float(duration.sum())
    t = np.linspace(0, total, 10001)
    pos = samples(traj, t, 0)
    numeric = {k: traj[k] for k in ('order','duration','coef_x','coef_y','coef_z')}
    return {'pieces': len(duration), 'duration': total,
            'sampled_path_length': float(np.linalg.norm(np.diff(pos, axis=0), axis=1).sum()),
            'start_position': pos[0].tolist(), 'end_position': pos[-1].tolist(),
            'numeric_sha256': hashlib.sha256(json.dumps(numeric, sort_keys=True).encode()).hexdigest()}


def compare_trajectory(a, b, args):
    sa, sb = trajectory_summary(a), trajectory_summary(b)
    result = {'before': sa, 'after': sb, 'numeric_exact': sa['numeric_sha256'] == sb['numeric_sha256']}
    result['duration_abs_difference'] = abs(sa['duration'] - sb['duration'])
    result['path_length_abs_difference'] = abs(sa['sampled_path_length'] - sb['sampled_path_length'])
    result['coefficient_checks'] = {}
    coefficient_ok = a['order'] == b['order']
    for key in ('duration', 'coef_x', 'coef_y', 'coef_z'):
        va, vb = np.asarray(a[key]), np.asarray(b[key])
        equal_shape = va.shape == vb.shape
        ok = equal_shape and bool(np.allclose(va, vb, atol=args.coefficient_atol, rtol=args.coefficient_rtol))
        coefficient_ok = coefficient_ok and ok
        result['coefficient_checks'][key] = {'same_shape': equal_shape, 'within_tolerance': ok,
                 'max_abs_difference': float(np.max(np.abs(va-vb))) if equal_shape else None}
    common_t = np.linspace(0, min(sa['duration'], sb['duration']), args.samples)
    normalized_a = np.linspace(0, sa['duration'], args.samples)
    normalized_b = np.linspace(0, sb['duration'], args.samples)
    result['common_time_max_norm_difference'] = {}
    result['normalized_time_max_norm_difference'] = {}
    sample_ok = True
    for derivative, name, tolerance in [(0,'position',args.position_atol), (1,'velocity',args.velocity_atol), (2,'acceleration',args.acceleration_atol)]:
        difference = float(np.linalg.norm(samples(a, common_t, derivative)-samples(b, common_t, derivative), axis=1).max())
        normalized_difference = float(np.linalg.norm(samples(a, normalized_a, derivative)-samples(b, normalized_b, derivative), axis=1).max())
        result['common_time_max_norm_difference'][name] = difference
        result['normalized_time_max_norm_difference'][name] = normalized_difference
        sample_ok = sample_ok and difference <= tolerance and normalized_difference <= tolerance
    result['pass'] = bool(coefficient_ok and sample_ok and result['duration_abs_difference'] <= args.duration_atol and result['path_length_abs_difference'] <= args.path_length_atol)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('before')
    parser.add_argument('after')
    parser.add_argument('--output')
    parser.add_argument('--samples', type=int, default=10001)
    parser.add_argument('--coefficient-atol', type=float, default=1e-6)
    parser.add_argument('--coefficient-rtol', type=float, default=1e-6)
    parser.add_argument('--duration-atol', type=float, default=1e-4)
    parser.add_argument('--position-atol', type=float, default=1e-3)
    parser.add_argument('--velocity-atol', type=float, default=1e-4)
    parser.add_argument('--acceleration-atol', type=float, default=1e-4)
    parser.add_argument('--path-length-atol', type=float, default=1e-3)
    args = parser.parse_args()
    a,b = [json.loads(Path(path).read_text()) for path in (args.before, args.after)]
    result = {'before_file': args.before, 'after_file': args.after,
              'units': 'Published PolyTraj units; no coordinate/unit conversion performed',
              'tolerances': {key: value for key, value in vars(args).items() if key.endswith(('atol','rtol'))},
              'sample_count': args.samples,
              'command_equal_ignoring_identity': without_keys(last(a,'commands'), {'mission_id','sequence'}) == without_keys(last(b,'commands'), {'mission_id','sequence'}),
              'environment': {}, 'counts': {key: [len(a[key]),len(b[key])] for key in ('commands','trajectories','initial_trajectories','execution_control')}}
    for key in ('terrain_ready','obstacles','risk_zones'):
        va = without_keys(last(a,key), {'header'}) if a[key] else None
        vb = without_keys(last(b,key), {'header'}) if b[key] else None
        result['environment'][key+'_equal'] = va == vb
    result['final'] = compare_trajectory(last(a,'trajectories'),last(b,'trajectories'),args)
    if a['initial_trajectories'] and b['initial_trajectories']:
        result['initial'] = compare_trajectory(last(a,'initial_trajectories'),last(b,'initial_trajectories'),args)
    result['pass'] = bool(result['command_equal_ignoring_identity'] and all(result['environment'].values()) and result['final']['pass'] and result.get('initial', {'pass': True})['pass'] and all(x == y for x,y in result['counts'].values()))
    rendered = json.dumps(result, indent=2)
    print(rendered)
    if args.output:
        Path(args.output).write_text(rendered+'\n')
    return 0 if result['pass'] else 1


if __name__ == '__main__':
    sys.exit(main())
