import json
from pxdex.dh15 import DexH15Control
h=DexH15Control()
ok,a=h.openUsbDevice('/dev/ttyACM1')
if not ok: raise SystemExit('No right USB connection')
try:
    print(json.dumps({'sn':h.getDeviceSN(a),'type':str(h.getHandType(a)),'enabled':h.isMotorEnabled(a),'status':h.checkHandStatus(a)}),flush=True)
    pts=h.getPressPositionForce(a)
    print(json.dumps({'count':len(pts),'points':[{'model':p.model_name,'xyz':[p.x,p.y,p.z],'force_raw':p.force_value} for p in pts]}),flush=True)
finally:
    h.disconnectUsb()
