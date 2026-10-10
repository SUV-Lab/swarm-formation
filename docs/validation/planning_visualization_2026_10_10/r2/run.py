from pathlib import Path
import datetime,json,os,signal,subprocess,time
root=Path('/tmp/mmp_stage4_visualization_20261010/r2_consumer')
assert os.environ.get('ROS_DOMAIN_ID')=='84'
summary={'started_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'domain':84,'checks':{},'modes':{}}
def run(name,cmd,timeout=300,env=None):
 if name == 'connected_run' and os.environ.get('MMP_REUSE_CONNECTED_COMPLETE') == '1':
  previous=json.loads((root/'attempt1_missing_comparator/summary.json').read_text())
  assert previous['checks'][name]['exit_code']==0 and (root/'connected_result.json').exists()
  summary['checks'][name]=dict(previous['checks'][name],reused_completed_attempt='attempt1_missing_comparator/summary.json')
  print(name,'reuse completed successful solve',flush=True);return
 start=time.monotonic()
 with (root/(name+'.log')).open('w') as out:
  try:rc=subprocess.run(cmd,cwd=root,stdout=out,stderr=subprocess.STDOUT,timeout=timeout,env=env).returncode
  except subprocess.TimeoutExpired:rc=124
 summary['checks'][name]={'command':cmd,'exit_code':rc,'elapsed_s':time.monotonic()-start};print(name,rc,flush=True)
 if rc:raise RuntimeError(name+' failed')
try:
 run('configure',['cmake','-S',str(root),'-B',str(root/'build'),'-DCMAKE_BUILD_TYPE=Release'])
 run('build',['cmake','--build',str(root/'build'),'-j2'])
 reference=json.loads(Path('/tmp/mmp_next_modularization_20261010/baseline/r2_consumer/result.json').read_text())
 for mode in ('connected','disconnected'):
  terrain=None;environment=os.environ.copy();environment.pop('MMP_VALUE_INPUT',None);environment.pop('MMP_VISUALIZATION_OFF',None)
  if mode=='disconnected':environment['MMP_VISUALIZATION_OFF']='1'
  result=root/(mode+'_result.json')
  try:
   with (root/(mode+'_terrain_loader.log')).open('w') as log:
    terrain=subprocess.Popen(['ros2','run','mmp_terrain','terrain_publisher','--ros-args','-p','world:=none'],cwd=root,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    summary['modes'][mode]={'own_loader_pid':terrain.pid}
    run(mode+'_run',[str(root/'build/r2_external_consumer'),'/ws/install/path_manager/share/path_manager/config/optimizer_params.yaml','/ws/install/path_manager/share/path_manager/config/drone_hardware.yaml','/ws/src/mmp_terrain/data/scenarios/missions/r2_ridge_crossing.yaml',str(result)],900,environment)
    run(mode+'_wire_compare',['python3',str(root/'compare.py'),'/tmp/mmp_next_modularization_20261010/baseline/gui/mission.json',str(result),str(root/(mode+'_wire_comparison.json'))])
    current=json.loads(result.read_text());exact={key:reference[key]==current.get(key) for key in reference}
    summary['modes'][mode]['field_exact']=exact;summary['modes'][mode]['pass']=all(exact.values())
    if not all(exact.values()):raise RuntimeError(mode+' differs from baseline full owned result')
  finally:
   if terrain is not None and terrain.poll() is None:
    os.killpg(terrain.pid,signal.SIGINT)
    try:terrain.wait(timeout=15)
    except subprocess.TimeoutExpired:os.killpg(terrain.pid,signal.SIGTERM);terrain.wait(timeout=10)
 summary['connected_disconnected_exact']=json.loads((root/'connected_result.json').read_text())==json.loads((root/'disconnected_result.json').read_text())
except Exception as e:summary['error']=str(e)
finally:
 summary['finished_utc']=datetime.datetime.now(datetime.timezone.utc).isoformat();summary['pass']='error' not in summary and summary.get('connected_disconnected_exact',False)
 (root/'summary.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary,indent=2),flush=True)
raise SystemExit(0 if summary['pass'] else 1)
