import ctypes as C
import os, signal, subprocess
from pathlib import Path

lib=C.CDLL('libX11.so.6')
lib.XOpenDisplay.argtypes=[C.c_char_p]; lib.XOpenDisplay.restype=C.c_void_p
d=lib.XOpenDisplay(b':1')
assert d
lib.XDefaultRootWindow.argtypes=[C.c_void_p]; lib.XDefaultRootWindow.restype=C.c_ulong
lib.XCreateSimpleWindow.argtypes=[C.c_void_p,C.c_ulong,C.c_int,C.c_int,C.c_uint,C.c_uint,C.c_uint,C.c_ulong,C.c_ulong]
lib.XCreateSimpleWindow.restype=C.c_ulong
win=lib.XCreateSimpleWindow(d,lib.XDefaultRootWindow(d),6000,0,1900,1500,0,0,0)
class Attributes(C.Structure):
    _fields_=[('background_pixmap',C.c_ulong),('background_pixel',C.c_ulong),('border_pixmap',C.c_ulong),('border_pixel',C.c_ulong),('bit_gravity',C.c_int),('win_gravity',C.c_int),('backing_store',C.c_int),('backing_planes',C.c_ulong),('backing_pixel',C.c_ulong),('save_under',C.c_int),('event_mask',C.c_long),('do_not_propagate_mask',C.c_long),('override_redirect',C.c_int),('colormap',C.c_ulong),('cursor',C.c_ulong)]
attrs=Attributes(); attrs.override_redirect=1
lib.XChangeWindowAttributes.argtypes=[C.c_void_p,C.c_ulong,C.c_ulong,C.POINTER(Attributes)]
lib.XChangeWindowAttributes(d,win,1<<9,C.byref(attrs))
lib.XMapWindow.argtypes=[C.c_void_p,C.c_ulong]; lib.XMapWindow(d,win)
lib.XFlush.argtypes=[C.c_void_p]; lib.XFlush(d)
def stop(*_): raise KeyboardInterrupt
signal.signal(signal.SIGTERM,stop)
process=subprocess.Popen(['Xephyr',':99','-parent',str(win),'-screen','1900x1500','-ac','-noreset','-no-host-grab','-nolisten','tcp'],env=dict(os.environ,DISPLAY=':1'),stdout=open('/tmp/mmp_isolated_display.log','w'),stderr=subprocess.STDOUT)
Path('/tmp/mmp_isolated_display.pid').write_text(str(os.getpid()))
print('isolated display :99; parent outside desktop; Xephyr PID',process.pid,flush=True)
try:
    process.wait()
finally:
    process.terminate()
    process.wait(timeout=10)
    lib.XCloseDisplay.argtypes=[C.c_void_p]; lib.XCloseDisplay(d)
