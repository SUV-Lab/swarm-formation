#!/usr/bin/env python3
"""Live ROS smoke: identical empty zones -> no abort; changed zone -> abort;
restore empty environment; original mission input/new identity -> fresh plan.
Intended only for the explicitly authorized R2 baseline/refactored test launch.
"""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.set_message import set_message_fields
from mmp_mission_msgs.msg import TrajectoryCommand, RiskZoneArray, RiskZoneSpec
from mmp_traj_msgs.msg import PolyTraj, TrajectoryExecutionControl
from rcl_interfaces.msg import Log


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('original_capture')
    p.add_argument('output')
    p.add_argument('--timeout', type=float, default=900.)
    args = p.parse_args()
    original = json.loads(Path(args.original_capture).read_text())
    original_command = original['commands'][-1]['message']
    original_traj = original['trajectories'][-1]['message']
    if original.get('risk_zones'):
        assert original['risk_zones'][-1]['message']['zones'] == [], 'This probe requires the R2 empty zone environment'
    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    started = time.monotonic()
    result = {'schema': 1, 'original_capture': args.original_capture, 'original_trajectory_id':original_traj['trajectory_id'],
              'checks':{},'events':[], 'controls':[], 'live_trajectories':[], 'live_initial_trajectories':[], 'rosout':[]}
    stage = 'setup'
    old_latched = []
    restore_needed = False
    def save():
        tmp = out.with_suffix(out.suffix+'.tmp')
        tmp.write_text(json.dumps(result, indent=2)+'\n')
        tmp.replace(out)
    def event(text):
        result['events'].append({'elapsed_s':time.monotonic()-started,'stage':stage,'detail':text})
        print(stage+': '+text,flush=True)
        save()
    def receive(key):
        def callback(msg):
            item = {'elapsed_s':time.monotonic()-started,'stage':stage,'message':message_to_ordereddict(msg)}
            result[key].append(item)
            if key != 'rosout':
                event('received '+key+' id='+str(getattr(msg,'trajectory_id','')))
        return callback
    rclpy.init()
    node = Node('mmp_modularization_abort_recovery_probe')
    reliable = QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.VOLATILE)
    latched = QoSProfile(depth=5,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscriptions = [
        node.create_subscription(PolyTraj,'/planning/trajectory',lambda msg:old_latched.append(message_to_ordereddict(msg)),latched),
        node.create_subscription(PolyTraj,'/planning/trajectory',receive('live_trajectories'),reliable),
        node.create_subscription(PolyTraj,'/planning/initial_trajectory',receive('live_initial_trajectories'),reliable),
        node.create_subscription(TrajectoryExecutionControl,'/planning/execution_control',receive('controls'),reliable),
        node.create_subscription(Log,'/rosout',receive('rosout'),reliable),
    ]
    zones_pub = node.create_publisher(RiskZoneArray,'/mission/risk_zones',latched)
    command_pub = node.create_publisher(TrajectoryCommand,'/mission/trajectory_command',reliable)
    def spin_for(seconds):
        end = time.monotonic()+seconds
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=min(.2,max(0,end-time.monotonic())))
    def wait_until(predicate,timeout,description):
        deadline=time.monotonic()+timeout
        next_progress=time.monotonic()+30.
        while not predicate():
            if time.monotonic()>deadline:
                raise RuntimeError('Timed out waiting for '+description)
            rclpy.spin_once(node,timeout_sec=.2)
            if time.monotonic()>next_progress:
                event('waiting for '+description)
                next_progress+=30.
    empty=RiskZoneArray(replace=True,zones=[])
    try:
        wait_until(lambda: old_latched and zones_pub.get_subscription_count()>0 and command_pub.get_subscription_count()>0,20.,'planner discovery and latched trajectory')
        spin_for(1.)
        result['current_trajectory_id']=old_latched[-1]['trajectory_id']
        assert old_latched[-1]['trajectory_id']==original_traj['trajectory_id'], 'Current trajectory differs from supplied baseline'
        assert not result['controls'], 'Unexpected live control before probe'
        event('connected; current trajectory id='+str(result['current_trajectory_id']))
        stage='identical_empty_zones'
        zones_pub.publish(empty)
        ack=zones_pub.wait_for_all_acked(Duration(seconds=5.))
        assert ack, 'Identical-zone publish was not acknowledged'
        spin_for(3.)
        result['checks']['identical_empty_zone_publish_acknowledged']=bool(ack)
        result['checks']['identical_empty_zones_no_abort']=len(result['controls'])==0
        assert result['checks']['identical_empty_zones_no_abort'], 'Identical empty zone republish aborted the flight'
        event('PASS: reliable empty-zone republish produced no ABORT over 3 seconds')
        stage='changed_far_zone'
        z=RiskZoneSpec()
        z.center.x=1.e6
        z.center.y=1.e6
        z.center.z=1.e6
        z.reach=1.
        z.peak=.5
        changed=RiskZoneArray(replace=True,zones=[z])
        result['changed_zone_message']=message_to_ordereddict(changed)
        restore_needed=True
        zones_pub.publish(changed)
        assert zones_pub.wait_for_all_acked(Duration(seconds=5.)), 'Changed-zone publish was not acknowledged'
        wait_until(lambda: len(result['controls'])>0,20.,'current-trajectory ABORT')
        abort=result['controls'][-1]['message']
        result['checks']['changed_zone_abort_exact_identity']=abort['trajectory_id']==result['current_trajectory_id']
        result['checks']['changed_zone_action_abort']=abort['action']==TrajectoryExecutionControl.ACTION_ABORT
        result['checks']['changed_zone_reason_policy_unevaluated']=abort['reason']==TrajectoryExecutionControl.REASON_POLICY_UNEVALUATED
        assert all(result['checks'].values()),'ABORT did not match current trajectory/action/reason'
        event('PASS: exact trajectory ABORT with POLICY_UNEVALUATED')
        stage='restore_empty_zones'
        controls_before=len(result['controls'])
        zones_pub.publish(empty)
        assert zones_pub.wait_for_all_acked(Duration(seconds=5.)), 'Zone restore publish was not acknowledged'
        spin_for(2.)
        restore_needed=False
        result['checks']['clear_after_abort_no_second_abort']=len(result['controls'])==controls_before
        assert result['checks']['clear_after_abort_no_second_abort'], 'Unexpected second ABORT after clearing already withdrawn flight'
        stage='recovery_mission'
        command=TrajectoryCommand()
        recovery_input=copy.deepcopy(original_command)
        recovery_input['sequence']+=1
        recovery_input['mission_id']+=':modularization-recovery'
        set_message_fields(command,recovery_input)
        result['recovery_command']=message_to_ordereddict(command)
        result['checks']['same_recovery_input_except_identity']=all(result['recovery_command'][key]==value for key,value in original_command.items() if key not in ('sequence','mission_id'))
        command_pub.publish(command)
        assert command_pub.wait_for_all_acked(Duration(seconds=5.)), 'Recovery mission publish was not acknowledged'
        event('recovery command delivered; waiting for fresh trajectory')
        wait_until(lambda: any(e['stage']=='recovery_mission' and e['message']['trajectory_id']>result['current_trajectory_id'] for e in result['live_trajectories']),args.timeout,'recovery trajectory')
        spin_for(5.)
        recovered=result['live_trajectories'][-1]['message']
        result['recovered_trajectory_id']=recovered['trajectory_id']
        result['checks']['recovery_fresh_identity']=recovered['trajectory_id']>result['current_trajectory_id']
        result['checks']['recovery_positive_duration']=bool(recovered['duration']) and all(d>0 for d in recovered['duration'])
        keys=('order','duration','coef_x','coef_y','coef_z')
        result['recovery_numeric_exact_vs_original']=all(recovered[key]==original_traj[key] for key in keys)
        result['recovered_numeric_sha256']=hashlib.sha256(json.dumps({k:recovered[k] for k in keys},sort_keys=True).encode()).hexdigest()
        result['checks']['exactly_one_abort']=len(result['controls'])==1
        result['pass']=all(result['checks'].values())
        recovered_capture=copy.deepcopy(original)
        recovered_capture['commands']=[{'elapsed_s':0,'message':result['recovery_command']}]
        recovered_capture['trajectories']=result['live_trajectories']
        recovered_capture['initial_trajectories']=result['live_initial_trajectories']
        recovered_capture['execution_control']=[]
        recovered_capture['rosout']=result['rosout']
        recovered_capture['probe_note']='Environment entries originate in original capture; script explicitly restored empty risk-zone environment before recovery.'
        recovered_path=out.with_name('recovery_mission.json')
        recovered_path.write_text(json.dumps(recovered_capture,indent=2)+'\n')
        result['recovered_capture']=str(recovered_path)
        event('PASS: recovered with fresh trajectory id='+str(recovered['trajectory_id'])+'; numeric_exact_vs_original='+str(result['recovery_numeric_exact_vs_original']))
    except Exception as exc:
        result['pass']=False
        result['error']=repr(exc)
        event('FAILED: '+repr(exc))
    finally:
        if restore_needed:
            zones_pub.publish(empty)
            zones_pub.wait_for_all_acked(Duration(seconds=5.))
            spin_for(1.)
            result['cleanup_restored_empty_zones']=True
        result['elapsed_s']=time.monotonic()-started
        save()
        node.destroy_node()
        rclpy.shutdown()
    return 0 if result.get('pass') else 1


if __name__=='__main__':
    sys.exit(main())
