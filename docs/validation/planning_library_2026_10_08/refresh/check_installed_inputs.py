from pathlib import Path
import datetime,hashlib,json,platform,subprocess
out=Path('/tmp/mmp_modularization_finalize')
pairs=[]
for name in ['path_manager','path_optimizer','path_planner','mmp_vehicle_dynamics']:
 source=Path('/ws/src/mmp_vehicle_dynamics' if name=='mmp_vehicle_dynamics' else '/ws/src/mmp_path_planning/src/'+name)
 for directory in ['include','config']:
  srcroot=source/directory
  if not srcroot.exists(): continue
  import os
  for parent,dirs,names in os.walk(srcroot,followlinks=False):
   dirs[:]=sorted(d for d in dirs if d not in {'reports','experiments','.git','__pycache__'})
   for basename in sorted(names):
    src=Path(parent)/basename
    if src.is_symlink():raise RuntimeError('Unexpected source symlink')
    rel=src.relative_to(srcroot)
    dst=(Path('/ws/install')/name/('include' if directory=='include' else 'share/'+name+'/config'))/rel
    if not src.is_file(): continue
    entry={'source':str(src),'installed':str(dst),'source_sha256':hashlib.sha256(src.read_bytes()).hexdigest(),'installed_sha256':hashlib.sha256(dst.read_bytes()).hexdigest() if dst.is_file() else None}
    entry['equal']=entry['source_sha256']==entry['installed_sha256'];pairs.append(entry)
summary={'created_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'file_count':len(pairs),'pass':all(e['equal'] for e in pairs),'files':pairs}
(out/'installed_input_comparison.json').write_text(json.dumps(summary,indent=2)+'\n')
metadata={'created_utc':summary['created_utc'],'uname':platform.uname()._asdict(),'tools':{},'cmake':{}}
for name,argv in [('compiler',['c++','--version']),('cmake',['cmake','--version']),('python',['python3','--version'])]:
 metadata['tools'][name]=subprocess.check_output(argv,text=True).strip()
for name in ['path_manager','path_optimizer','path_planner','mmp_vehicle_dynamics']:
 cache=Path('/ws/build')/name/'CMakeCache.txt'
 lines=[s for s in cache.read_text().splitlines() if s.startswith(('CMAKE_BUILD_TYPE:','CMAKE_CXX_COMPILER:','CMAKE_CXX_FLAGS','CMAKE_GENERATOR:','BUILD_TESTING:','MMP_SANITIZE:','CMAKE_HOME_DIRECTORY:'))]
 metadata['cmake'][name]=lines
(out/'build_environment.json').write_text(json.dumps(metadata,indent=2)+'\n')
print(json.dumps({'installed_input_files':len(pairs),'pass':summary['pass'],'mismatches':[p for p in pairs if not p['equal']]}))
raise SystemExit(0 if summary['pass'] else 1)
