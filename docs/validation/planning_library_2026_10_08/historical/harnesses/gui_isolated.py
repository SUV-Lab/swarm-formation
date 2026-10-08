import ctypes as c, sys, time, os
x=c.CDLL('libX11.so.6'); t=c.CDLL('libXtst.so.6')
x.XOpenDisplay.argtypes=[c.c_char_p]; x.XOpenDisplay.restype=c.c_void_p
d=x.XOpenDisplay(os.environ.get('DISPLAY', ':1').encode())
assert d
x.XStringToKeysym.argtypes=[c.c_char_p]; x.XStringToKeysym.restype=c.c_ulong
x.XKeysymToKeycode.argtypes=[c.c_void_p,c.c_ulong]; x.XKeysymToKeycode.restype=c.c_uint
x.XFlush.argtypes=[c.c_void_p]
t.XTestFakeKeyEvent.argtypes=[c.c_void_p,c.c_uint,c.c_int,c.c_ulong]
t.XTestFakeButtonEvent.argtypes=[c.c_void_p,c.c_uint,c.c_int,c.c_ulong]
t.XTestFakeMotionEvent.argtypes=[c.c_void_p,c.c_int,c.c_int,c.c_int,c.c_ulong]
def key(name,down):
 code=x.XKeysymToKeycode(d,x.XStringToKeysym(name.encode()))
 assert code, name
 t.XTestFakeKeyEvent(d,code,down,0); x.XFlush(d)
def tap(name):
 key(name,1); key(name,0)
if sys.argv[1]=='click':
 t.XTestFakeMotionEvent(d,-1,int(sys.argv[2]),int(sys.argv[3]),0)
 t.XTestFakeButtonEvent(d,1,1,0);t.XTestFakeButtonEvent(d,1,0,0);x.XFlush(d)
elif sys.argv[1]=='type':
 for ch in sys.argv[2]:
  shift=ch=='_' or ch.isupper()
  if shift:key('Shift_L',1)
  tap({'/':'slash','.':'period','_':'minus','-':'minus',' ':'space'}.get(ch,ch.lower()))
  if shift:key('Shift_L',0)
  time.sleep(.003)
elif sys.argv[1]=='key':
 for name in sys.argv[2:]: tap(name)
elif sys.argv[1]=='select_all':
 key('Control_L',1);tap('a');key('Control_L',0)
