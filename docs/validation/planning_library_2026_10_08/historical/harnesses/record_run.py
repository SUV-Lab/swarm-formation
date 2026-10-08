#!/usr/bin/env python3
"""Read-only live ROS capture. Start before RViz Run; final QoS is VOLATILE.
Usage: python3 record_run.py OUTPUT.json [--timeout 900] [--grace 5]
The readiness line means subscribers exist; it does not drive the GUI or mission.
"""
import argparse
import json
import os
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rosidl_runtime_py.convert import message_to_ordereddict
from mmp_mission_msgs.msg import TrajectoryCommand, DynamicObstacleArray, RiskZoneArray
from mmp_traj_msgs.msg import PolyTraj, TrajectoryExecutionControl
from std_msgs.msg import Float64MultiArray
from rcl_interfaces.msg import Log
from nav_msgs.msg import Path as NavPath
from visualization_msgs.msg import Marker


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output')
    parser.add_argument('--timeout', type=float, default=900.0)
    parser.add_argument('--grace', type=float, default=5.0)
    parser.add_argument('--ready-file')
    args = parser.parse_args()
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    started = time.monotonic()
    capture = {'schema': 1, 'capture_started_unix': time.time(),
               'ros_domain_id': os.environ.get('ROS_DOMAIN_ID', '0'),
               'final_qos': 'RELIABLE/VOLATILE; only accepted after live command',
               'commands': [], 'terrain_ready': [], 'obstacles': [], 'risk_zones': [],
               'trajectories': [], 'initial_trajectories': [], 'execution_control': [],
               'rosout': [], 'front_end_paths': [], 'front_end_tubes': [], 'ignored_precommand_trajectories': 0}
    first_final = None
    def save():
        tmp = output.with_suffix(output.suffix + '.tmp')
        tmp.write_text(json.dumps(capture, indent=2, allow_nan=True) + '\n')
        tmp.replace(output)
    rclpy.init()
    node = Node('mmp_modularization_capture_' + str(os.getpid()))
    volatile = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
    latched = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL)
    def callback(key):
        def receive(msg):
            nonlocal first_final
            if key in ('trajectories', 'initial_trajectories', 'front_end_paths', 'front_end_tubes') and not capture['commands']:
                capture['ignored_precommand_trajectories'] += 1
                return
            entry = {'elapsed_s': time.monotonic() - started,
                     'message': message_to_ordereddict(msg)}
            capture[key].append(entry)
            if key == 'trajectories' and first_final is None:
                first_final = time.monotonic()
            if key != 'rosout':
                print('CAPTURE ' + key + ' elapsed=' + str(round(entry['elapsed_s'], 3)), flush=True)
                save()
        return receive
    subscriptions = []
    for typ, topic, key, qos in [
        (TrajectoryCommand, '/mission/trajectory_command', 'commands', volatile),
        (Float64MultiArray, '/planning/terrain_ready', 'terrain_ready', latched),
        (DynamicObstacleArray, '/mission/obstacles', 'obstacles', latched),
        (RiskZoneArray, '/mission/risk_zones', 'risk_zones', latched),
        (PolyTraj, '/planning/trajectory', 'trajectories', volatile),
        (PolyTraj, '/planning/initial_trajectory', 'initial_trajectories', volatile),
        (TrajectoryExecutionControl, '/planning/execution_control', 'execution_control', volatile),
        (Log, '/rosout', 'rosout', volatile),
        (NavPath, '/planning/front_end_path', 'front_end_paths', volatile),
        (Marker, '/viz/front_end_path', 'front_end_tubes', volatile),
    ]:
        subscriptions.append(node.create_subscription(typ, topic, callback(key), qos))
    save()
    print('RECORDER_READY output=' + str(output), flush=True)
    if args.ready_file:
        Path(args.ready_file).write_text(str(output) + '\n')
    try:
        while rclpy.ok() and time.monotonic() - started < args.timeout:
            rclpy.spin_once(node, timeout_sec=0.2)
            if first_final is not None and time.monotonic() - first_final >= args.grace:
                capture['completion'] = 'trajectory_received'
                break
        else:
            capture['completion'] = 'timeout_without_final' if first_final is None else 'timeout'
    except KeyboardInterrupt:
        capture['completion'] = 'interrupted'
    finally:
        capture['capture_elapsed_s'] = time.monotonic() - started
        save()
        node.destroy_node()
        rclpy.shutdown()
    print('RECORDER_DONE completion=' + capture['completion'] +
          ' commands=' + str(len(capture['commands'])) +
          ' final=' + str(len(capture['trajectories'])) +
          ' initial=' + str(len(capture['initial_trajectories'])), flush=True)
    return 0 if capture['trajectories'] else 2


if __name__ == '__main__':
    sys.exit(main())
