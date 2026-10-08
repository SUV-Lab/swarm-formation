#!/usr/bin/env python3
"""Run as user/domain83 after Stage C build/install notification."""
from pathlib import Path
import datetime,hashlib,json,os,shutil,subprocess,time
root=Path('/tmp/mmp_modularization_ext_c/consumer')
base=Path('/tmp/mmp_modularization_ext_b/after')
after=root.parent/'after'
after.mkdir(parents=True,exist_ok=True)
assert os.environ.get('ROS_DOMAIN_ID')=='83'
summary={'started_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'ros_domain_id':83,'checks':{}}
def run(name,cmd,env=None):
    begin=time.monotonic()
    with (root/(name+'.log')).open('w') as f:
        try: rc=subprocess.run(cmd,cwd=root,env=env,stdout=f,stderr=subprocess.STDOUT,timeout=300).returncode
        except subprocess.TimeoutExpired: rc=124
    summary['checks'][name]={'command':cmd,'exit_code':rc,'elapsed_s':time.monotonic()-begin,'log':str(root/(name+'.log'))}
    (root/'after_summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(name+' rc='+str(rc),flush=True)
    return rc==0
clean=os.environ.copy()
for k in ('CPATH','CPLUS_INCLUDE_PATH','C_INCLUDE_PATH','OBJC_INCLUDE_PATH'): clean.pop(k,None)
run('value_headers',['/usr/bin/c++','-std=c++17','-fsyntax-only','-H','-I/ws/install/path_manager/include','-I/ws/install/path_optimizer/include','-I/ws/install/path_planner/include','-I/ws/install/mmp_vehicle_dynamics/include','-I/usr/include/eigen3',str(root/'value_headers.cpp')],clean)
summary['checks']['value_headers']['ros_dependency_seen_in_header_trace']='/opt/ros/' in (root/'value_headers.log').read_text()

core_sources=[]
summary['standalone_core_source_sha256']={}
for relative in ('src/mission_planner.cpp','src/planning_environment.cpp','test/mission_planner_core_test.cpp'):
    source=Path('/ws/src/mmp_path_planning/src/path_manager')/relative
    target=root/('standalone_'+source.name)
    shutil.copy2(source,target)
    core_sources.append(str(target))
    summary['standalone_core_source_sha256'][relative]=hashlib.sha256(source.read_bytes()).hexdigest()
if run('mission_core_compile',['/usr/bin/c++','-std=c++17','-O2','-H','-I/ws/install/path_manager/include','-I/ws/install/path_optimizer/include','-I/ws/install/mmp_vehicle_dynamics/include','-I/usr/include/eigen3',*core_sources,'-o',str(root/'standalone_mission_core_test')],clean):
    run('mission_core_run',[str(root/'standalone_mission_core_test')],clean)
summary['checks']['mission_core_compile']['ros_dependency_seen_in_header_trace']='/opt/ros/' in (root/'mission_core_compile.log').read_text()

geometry_source=Path('/ws/src/mmp_path_planning/src/path_manager/src/mission_geometry.cpp')
geometry_test=Path('/ws/src/mmp_path_planning/src/path_manager/test/mission_geometry_test.cpp')
shutil.copy2(geometry_source,root/'standalone_mission_geometry.cpp')
shutil.copy2(geometry_test,root/'standalone_mission_geometry_test.cpp')
summary['standalone_geometry_source_sha256']=hashlib.sha256(geometry_source.read_bytes()).hexdigest()
summary['standalone_geometry_test_sha256']=hashlib.sha256(geometry_test.read_bytes()).hexdigest()
if run('geometry_compile',['/usr/bin/c++','-std=c++17','-O2','-H','-I/ws/install/path_manager/include','-I/usr/include/eigen3',str(root/'standalone_mission_geometry.cpp'),str(root/'standalone_mission_geometry_test.cpp'),'-o',str(root/'standalone_geometry_test')],clean):
    run('geometry_run',[str(root/'standalone_geometry_test')],clean)
summary['checks']['geometry_compile']['ros_dependency_seen_in_header_trace']='/opt/ros/' in (root/'geometry_compile.log').read_text()

postprocessing_source=Path('/ws/src/mmp_path_planning/src/path_manager/src/route_postprocessing.cpp')
postprocessing_test=Path('/ws/src/mmp_path_planning/src/path_manager/test/route_postprocessing_test.cpp')
shutil.copy2(postprocessing_source,root/'standalone_route_postprocessing.cpp')
shutil.copy2(postprocessing_test,root/'standalone_route_postprocessing_test.cpp')
summary['standalone_postprocessing_source_sha256']=hashlib.sha256(postprocessing_source.read_bytes()).hexdigest()
summary['standalone_postprocessing_test_sha256']=hashlib.sha256(postprocessing_test.read_bytes()).hexdigest()
if run('postprocessing_compile',['/usr/bin/c++','-std=c++17','-O2','-H','-I/ws/install/path_manager/include','-I/ws/install/path_planner/include','-I/usr/include/eigen3',str(root/'standalone_route_postprocessing.cpp'),str(root/'standalone_route_postprocessing_test.cpp'),'-o',str(root/'standalone_postprocessing_test')],clean):
    run('postprocessing_run',[str(root/'standalone_postprocessing_test')],clean)
summary['checks']['postprocessing_compile']['ros_dependency_seen_in_header_trace']='/opt/ros/' in (root/'postprocessing_compile.log').read_text()


profile_source=Path('/ws/src/mmp_path_planning/src/path_optimizer/src/route_cost_profile.cpp')
profile_test=Path('/ws/src/mmp_path_planning/src/path_optimizer/test/route_cost_profile_test.cpp')
shutil.copy2(profile_source,root/'standalone_route_cost_profile.cpp')
shutil.copy2(profile_test,root/'standalone_route_cost_profile_test.cpp')
summary['standalone_route_cost_profile_source_sha256']=hashlib.sha256(profile_source.read_bytes()).hexdigest()
summary['standalone_route_cost_profile_test_sha256']=hashlib.sha256(profile_test.read_bytes()).hexdigest()
if run('route_cost_profile_compile',['/usr/bin/c++','-std=c++17','-O2','-H','-I/ws/install/path_optimizer/include','-I/usr/include/eigen3',str(root/'standalone_route_cost_profile.cpp'),str(root/'standalone_route_cost_profile_test.cpp'),'-o',str(root/'standalone_route_cost_profile_test')],clean):
    run('route_cost_profile_run',[str(root/'standalone_route_cost_profile_test')],clean)
summary['checks']['route_cost_profile_compile']['ros_dependency_seen_in_header_trace']='/opt/ros/' in (root/'route_cost_profile_compile.log').read_text()

search_source=Path('/ws/src/mmp_path_planning/src/path_manager/src/mission_route_search.cpp')
search_test=Path('/ws/src/mmp_path_planning/src/path_manager/test/mission_route_search_test.cpp')
shutil.copy2(search_source,root/'standalone_mission_route_search.cpp')
shutil.copy2(search_test,root/'standalone_mission_route_search_test.cpp')
summary['standalone_mission_route_search_source_sha256']=hashlib.sha256(search_source.read_bytes()).hexdigest()
summary['standalone_mission_route_search_test_sha256']=hashlib.sha256(search_test.read_bytes()).hexdigest()
if run('mission_route_search_compile',['/usr/bin/c++','-std=c++17','-O2','-H','-I/ws/install/path_manager/include','-I/usr/include/eigen3',str(root/'standalone_mission_route_search.cpp'),str(root/'standalone_mission_route_search_test.cpp'),'-o',str(root/'standalone_mission_route_search_test')],clean):
    run('mission_route_search_run',[str(root/'standalone_mission_route_search_test')],clean)
summary['checks']['mission_route_search_compile']['ros_dependency_seen_in_header_trace']='/opt/ros/' in (root/'mission_route_search_compile.log').read_text()

if run('configure_after',['cmake','-S',str(root),'-B',str(root/'build_after'),'-DCMAKE_BUILD_TYPE=Release','-DCMAKE_EXPORT_COMPILE_COMMANDS=ON']) and run('facade_header',['cmake','--build',str(root/'build_after'),'--target','facade_header_probe','-j2']) and run('build_after',['cmake','--build',str(root/'build_after'),'--target','exact_mission_consumer','-j2']):
    summary['comparisons']={}
    for mode in ('direct','default'):
        if run('run_'+mode,[str(root/'build_after/exact_mission_consumer'),'/ws/install/path_manager/share/path_manager/config/optimizer_params.yaml',mode,str(after/(mode+'.json'))]):
            a=json.loads((base/(mode+'.json')).read_text()); b=json.loads((after/(mode+'.json')).read_text())
            fields=('schema','mode','input','status','final','reference','phases','evaluation','identity')
            same={key:json.dumps(a[key],sort_keys=True)==json.dumps(b[key],sort_keys=True) for key in fields}
            summary['comparisons'][mode]={'same_fields_ignoring_times':same,'pass':all(same.values()),'final_sha256':hashlib.sha256(json.dumps(b['final'],sort_keys=True).encode()).hexdigest(),'reference_sha256':hashlib.sha256(json.dumps(b['reference'],sort_keys=True).encode()).hexdigest(),'final_piece_count':b['final']['piece_count'],'final_duration':b['final']['duration'],'reference_duration':b['reference']['duration']}
optimizer_root=root.parent/'optimizer_consumer'
optimizer_source=Path('/ws/src/mmp_path_planning/src/path_optimizer/test/optimizer_session_test.cpp')
shutil.copy2(optimizer_source,optimizer_root/'main.cpp')
summary['optimizer_consumer_source_sha256']=hashlib.sha256(optimizer_source.read_bytes()).hexdigest()
if run('optimizer_configure',['cmake','-S',str(optimizer_root),'-B',str(optimizer_root/'build'),'-DCMAKE_BUILD_TYPE=Release']) and run('optimizer_build',['cmake','--build',str(optimizer_root/'build'),'-j2']):
    run('optimizer_run',[str(optimizer_root/'build/optimizer_only_consumer'),'/ws/install/path_manager/share/path_manager/config/optimizer_params.yaml'])
optimizer_link=optimizer_root/'build/CMakeFiles/optimizer_only_consumer.dir/link.txt'
summary['optimizer_link_has_path_manager']=not optimizer_link.exists() or 'path_manager' in optimizer_link.read_text()
if optimizer_link.exists(): shutil.copy2(optimizer_link,root/'optimizer_link.txt')
model_root=root.parent/'model_consumer'
if run('model_configure',['cmake','-S',str(model_root),'-B',str(model_root/'build'),'-DCMAKE_BUILD_TYPE=Release']) and run('model_build',['cmake','--build',str(model_root/'build'),'-j2']):
    run('model_run',[str(model_root/'build/vehicle_model_consumer'),'/ws/install/path_manager/share/path_manager/config/optimizer_params.yaml'])
route_root=root.parent/'route_consumer'
route_source=Path('/ws/src/mmp_path_planning/src/path_manager/test/route_search_backend_test.cpp')
shutil.copy2(route_source,route_root/'main.cpp')
summary['route_consumer_source_sha256']=hashlib.sha256(route_source.read_bytes()).hexdigest()
if run('route_configure',['cmake','-S',str(route_root),'-B',str(route_root/'build'),'-DCMAKE_BUILD_TYPE=Release']) and run('route_build',['cmake','--build',str(route_root/'build'),'-j2']):
    run('route_run',[str(route_root/'build/route_search_consumer'),'/ws/install/path_manager/share/path_manager/config/optimizer_params.yaml'])
facade_log=root/'facade_header.log'
forbidden_headers=('path_manager/path_manager.h','path_optimizer/poly_traj_optimizer.h','path_planner/dyn_a_star.h','path_planner/sdf/')
summary['facade_header_isolation']={'source_includes_only':'path_manager/mission_planner.h','forbidden_headers':list(forbidden_headers),'found':[]}
if facade_log.exists():
    trace=facade_log.read_text()
    summary['facade_header_isolation']['found']=[header for header in forbidden_headers if header in trace]
summary['facade_header_isolation']['pass']=summary['checks'].get('facade_header',{}).get('exit_code')==0 and not summary['facade_header_isolation']['found']
summary['finished_utc']=datetime.datetime.now(datetime.timezone.utc).isoformat()
summary['pass']=all(name in summary['checks'] and summary['checks'][name]['exit_code']==0 for name in ('mission_core_compile','mission_core_run','route_configure','route_build','route_run','model_configure','model_build','model_run','value_headers','geometry_compile','geometry_run','postprocessing_compile','postprocessing_run','route_cost_profile_compile','route_cost_profile_run','mission_route_search_compile','mission_route_search_run','optimizer_configure','optimizer_build','optimizer_run','configure_after','facade_header','build_after','run_direct','run_default')) and not summary['checks']['mission_core_compile']['ros_dependency_seen_in_header_trace'] and not summary['checks']['value_headers']['ros_dependency_seen_in_header_trace'] and not summary['checks']['geometry_compile']['ros_dependency_seen_in_header_trace'] and not summary['checks']['postprocessing_compile']['ros_dependency_seen_in_header_trace'] and not summary['checks']['route_cost_profile_compile']['ros_dependency_seen_in_header_trace'] and not summary['checks']['mission_route_search_compile']['ros_dependency_seen_in_header_trace'] and not summary['optimizer_link_has_path_manager'] and summary['facade_header_isolation']['pass'] and all(summary.get('comparisons',{}).get(mode,{}).get('pass',False) for mode in ('direct','default'))
(root/'after_summary.json').write_text(json.dumps(summary,indent=2)+'\n')
print(json.dumps(summary,indent=2),flush=True)
raise SystemExit(0 if summary['pass'] else 1)
