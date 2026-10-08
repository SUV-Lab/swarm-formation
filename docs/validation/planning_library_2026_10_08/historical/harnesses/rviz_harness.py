#!/usr/bin/env python3
"""Drive the existing RViz panels and retain per-stage regression evidence."""
from pathlib import Path
import ctypes as C
import json, os, re, shlex, subprocess, sys, time
from collections import Counter

root = Path(sys.argv[1]); out = root / 'gui'; out.mkdir(parents=True, exist_ok=True)
baseline = Path('/tmp/mmp_modularization_ext_baseline/gui')
display=os.environ.get('MMP_GUI_DISPLAY',':1')
gui = '/tmp/mmp_modularization/gui_isolated.py'
def host(args, **kwargs):
    return subprocess.run(args, check=True, text=True, **kwargs)
def container(command, **kwargs):
    return host(['docker','exec','-u','user','mmp_dev','bash','-lc',command], **kwargs)
def read(path):
    return container('cat '+shlex.quote(str(path)), capture_output=True).stdout
def wait_for(path, text, timeout=120):
    start=time.monotonic()
    while time.monotonic()-start < timeout:
        try:
            data=read(path)
            if text in data: return data
        except subprocess.CalledProcessError:
            pass
        time.sleep(1)
    raise RuntimeError('timeout waiting for '+text+' in '+str(path))
def rviz_window(activate=False):
    tree=host(['xwininfo','-root','-tree'],env=dict(os.environ,DISPLAY=display),capture_output=True).stdout
    lines=[line for line in tree.splitlines() if ' - RViz"' in line and '("rviz2" "rviz2")' in line]
    if len(lines)!=1: raise RuntimeError('expected one RViz main window')
    match=re.search(r'(0x[0-9a-f]+).*? (\d+)x(\d+)[+-]\d+[+-]\d+\s+([+-]\d+)([+-]\d+)\s*$',lines[0])
    if not match: raise RuntimeError('cannot locate RViz window geometry')
    wid,width,height,left,top=[int(v,16) if i==0 else int(v) for i,v in enumerate(match.groups())]
    if (width,height)!=(1685,1371): raise RuntimeError(f'RViz panel layout changed: {width}x{height}')
    if activate:
        lib=C.CDLL('libX11.so.6')
        lib.XOpenDisplay.argtypes=[C.c_char_p]; lib.XOpenDisplay.restype=C.c_void_p
        connection=lib.XOpenDisplay(display.encode())
        lib.XDefaultRootWindow.argtypes=[C.c_void_p]; lib.XDefaultRootWindow.restype=C.c_ulong
        lib.XInternAtom.argtypes=[C.c_void_p,C.c_char_p,C.c_int]; lib.XInternAtom.restype=C.c_ulong
        class ClientMessage(C.Structure):
            _fields_=[('type',C.c_int),('serial',C.c_ulong),('send_event',C.c_int),('display',C.c_void_p),('window',C.c_ulong),('message_type',C.c_ulong),('format',C.c_int),('data',C.c_long*5)]
        class Event(C.Union):
            _fields_=[('client',ClientMessage),('pad',C.c_long*24)]
        event=Event(); event.client.type=33; event.client.send_event=1
        event.client.display=connection; event.client.window=wid
        event.client.message_type=lib.XInternAtom(connection,b'_NET_ACTIVE_WINDOW',0)
        event.client.format=32; event.client.data[0]=2
        lib.XSendEvent.argtypes=[C.c_void_p,C.c_ulong,C.c_int,C.c_long,C.POINTER(Event)]
        if display == ':1':
            lib.XSendEvent(connection,lib.XDefaultRootWindow(connection),0,(1<<19)|(1<<20),C.byref(event))
        else:
            lib.XMoveWindow.argtypes=[C.c_void_p,C.c_ulong,C.c_int,C.c_int]
            lib.XMoveWindow(connection,wid,0,0)
            lib.XRaiseWindow.argtypes=[C.c_void_p,C.c_ulong]; lib.XRaiseWindow(connection,wid)
            lib.XSetInputFocus.argtypes=[C.c_void_p,C.c_ulong,C.c_int,C.c_ulong]
            lib.XSetInputFocus(connection,wid,2,0)
            left=top=0
        lib.XFlush.argtypes=[C.c_void_p]; lib.XFlush(connection)
        lib.XCloseDisplay.argtypes=[C.c_void_p]; lib.XCloseDisplay(connection)
        time.sleep(.3)
    return left,top,left+width,top+height
def click(x,y):
    left,top,_,_=rviz_window(activate=True)
    host(['python3',gui,'click',str(left+x-1689),str(top+y-69)],env=dict(os.environ,DISPLAY=display))
def cp_back(name): host(['docker','cp','mmp_dev:'+str(out/name),str(out/name)],stdout=subprocess.DEVNULL)
def snapshot(name):
    from PIL import ImageGrab
    ImageGrab.grab(bbox=rviz_window(activate=True),xdisplay=display).save(str(out/name))

existing=container('ps -eo pid,ppid,args',capture_output=True).stdout
if any('/ws/install/path_manager/lib/path_manager/path_manager_node ' in l or '/opt/ros/humble/lib/rviz2/rviz2 ' in l for l in existing.splitlines()):
    raise RuntimeError('existing GUI/planner process: refusing to mix sessions')
container('mkdir -p '+shlex.quote(str(out)))
setup='export DISPLAY='+shlex.quote(display)+' LIBGL_ALWAYS_SOFTWARE=1 ROS_DOMAIN_ID=82; source /opt/ros/humble/setup.bash; source /ws/install/setup.bash; '
launch_cmd='echo $$ > '+shlex.quote(str(out/'launcher.pid'))+'; '+setup+'exec ros2 launch mmp_launch mmp.launch.py > '+shlex.quote(str(out/'launch.log'))+' 2>&1'
launch=subprocess.Popen(['docker','exec','-u','user','mmp_dev','bash','-lc',launch_cmd])
rec_cmd='echo $$ > '+shlex.quote(str(out/'recorder.pid'))+'; '+setup+'exec python3 /tmp/mmp_modularization_stage7/record_run.py '+shlex.quote(str(out/'mission.json'))+' --timeout 300 --grace 5 > '+shlex.quote(str(out/'recorder.log'))+' 2>&1'
recorder=subprocess.Popen(['docker','exec','-u','user','mmp_dev','bash','-lc',rec_cmd])
try:
    wait_for(out/'launch.log','TrajectoryMetricsPanel initialized')
    wait_for(out/'recorder.log','RECORDER_READY')
    click(1813,456)
    log=wait_for(out/'launch.log','PathManager initialized, waiting for trajectory command')
    planner_pid=int(re.search(r'path_manager_node-\d+\]: process started with pid \[(\d+)\]',log).group(1))
    planner_parent_pid=int(container('ps -o ppid= -p '+str(planner_pid),capture_output=True).stdout.strip())
    click(1880,923); host(['python3',gui,'select_all'],env=dict(os.environ,DISPLAY=display))
    host(['python3',gui,'type','/ws/src/mmp_terrain/data/scenarios/missions/r2_ridge_crossing.yaml'],env=dict(os.environ,DISPLAY=display))
    click(1926,981)
    wait_for(out/'recorder.log','RECORDER_DONE',timeout=180)
    recorder.wait(timeout=10)
    if recorder.returncode: raise RuntimeError('recorder failed')
    fds=container('ls -l /proc/'+str(planner_pid)+'/fd',capture_output=True).stdout
    runtime_log=re.search(r'-> (/ws/logs/runtime/[^\n]+)',fds).group(1)
    container('cp '+shlex.quote(runtime_log)+' '+shlex.quote(str(out/'planner_initial.log')))
    hashes=container('sha256sum /proc/'+str(planner_pid)+'/exe /ws/install/path_manager/lib/path_manager/path_manager_node',capture_output=True).stdout
    (out/'binary_hashes.txt').write_text(hashes)
    assert len(set(line.split()[0] for line in hashes.splitlines()))==1
    cp_back('mission.json'); cp_back('planner_initial.log'); snapshot('rviz_r2.png')
    with (out/'comparison.log').open('w') as log:
        host(['python3','/tmp/mmp_modularization/compare_runs.py',str(baseline/'mission.json'),str(out/'mission.json'),'--output',str(out/'comparison.json')],stdout=log)
    host(['python3','/tmp/mmp_modularization_stage7/compare_frontend.py',str(baseline/'mission.json'),str(out/'mission.json'),str(out/'frontend_comparison.json')],stdout=subprocess.DEVNULL)
    print('R2 generated and numeric/frontend comparison passed',flush=True)
    container(setup+'python3 /tmp/mmp_modularization/probe_abort.py '+shlex.quote(str(out/'mission.json'))+' '+shlex.quote(str(out/'abort.json'))+' > '+shlex.quote(str(out/'abort.log'))+' 2>&1',timeout=90)
    cp_back('abort.json'); cp_back('abort.log')
    abort=json.loads((out/'abort.json').read_text()); assert abort['pass'] and abort['recovery_numeric_exact_vs_original']
    container('cp '+shlex.quote(runtime_log)+' '+shlex.quote(str(out/'planner.log')))
    cp_back('planner.log'); snapshot('rviz_recovered.png')
    before=(baseline/'planner.log').read_text().splitlines(); after=(out/'planner.log').read_text().splitlines()
    checks={}
    for name,tags,ordered in [('evaluation',('[CHAIN-REPORT]','[FINAL-EVAL]'),True),('postprocessing',('[Z-DENOISE]','[A*-SHORTCUT]','[CORNER-FILLET]'),True),('cost_profile',('[ALT-CAP]','[OCCLUSION-CAP]','[RIDE]'),False)]:
        def select(lines): return [l.split('] ',2)[-1] for l in lines if any(t in l for t in tags)]
        b,a=select(before),select(after)
        check={'before_count':len(b),'after_count':len(a),'ordered':ordered,'pass':bool(b) and (b==a if ordered else Counter(b)==Counter(a))}
        (out/(name+'_comparison.json')).write_text(json.dumps(check,indent=2)+'\n'); checks[name]=check
        assert check['pass'],name
    checks['abort']={'pass':abort['pass'],'numeric_exact':abort['recovery_numeric_exact_vs_original']}
    checks['pass']=True; (out/'summary.json').write_text(json.dumps(checks,indent=2)+'\n')
    print('ABORT/recovery and evaluation comparisons passed',flush=True)
finally:
    if recorder.poll() is None:
        try:
            container('kill -INT '+str(int(read(out/'recorder.pid').strip())))
            recorder.wait(timeout=10)
        except Exception as error:
            print('recorder cleanup needs inspection:',error,flush=True)
    try:
        top=int(read(out/'launcher.pid').strip())
        ps=container('ps -eo pid,ppid,args',capture_output=True).stdout.splitlines()[1:]
        rows={int(p[0]):(int(p[1]),p[2]) for l in ps if len(p:=l.strip().split(None,2))==3}
        descendants={top}
        # The RViz panel deliberately starts a detached launcher. Its PID may
        # already have PPID 1, so the main launcher's tree alone is insufficient.
        for name in ('planner_pid','planner_parent_pid'):
            pid=locals().get(name)
            if pid in rows: descendants.add(pid)
        for _ in range(12): descendants.update(pid for pid,(parent,_) in rows.items() if parent in descendants)
        launchers=[pid for pid in descendants if pid in rows and 'ros2 launch ' in rows[pid][1]]
        if launchers: container('kill -INT '+' '.join(map(str,sorted(launchers,reverse=True))))
        launch.wait(timeout=30)
        # Some detached launchers inherit SIGINT ignored from the GUI. Reap
        # only processes whose PID and command still match this run's tree.
        current=container('ps -eo pid,ppid,args',capture_output=True).stdout.splitlines()[1:]
        remaining=[]
        for line in current:
            p=line.strip().split(None,2)
            if len(p)==3 and int(p[0]) in descendants and int(p[0]) in rows and p[2]==rows[int(p[0])][1]:
                remaining.append(int(p[0]))
        if remaining:
            container('kill -TERM '+' '.join(map(str,sorted(remaining,reverse=True))))
            time.sleep(1)
    except Exception as error:
        print('cleanup needs inspection:',error,flush=True)
    for name in ('launch.log','recorder.log'):
        try: cp_back(name)
        except Exception: pass
    for p in out.iterdir():
        if p.is_file(): host(['docker','cp',str(p),'mmp_dev:'+str(p)],stdout=subprocess.DEVNULL)
