"""Read status/version/faults only; no setters, enable or initialization."""
import json
from pxdex.dh15 import DexH15Control
h=DexH15Control();ok,a=h.openUsbDevice('/dev/ttyACM1')
if not ok:raise RuntimeError('USB open failed')
try:
 sn=h.getDeviceSN(a);assert sn=='H15R49G00039'
 result={'sn':sn,'enabled':h.isMotorEnabled(a),'status':h.checkHandStatus(a),
         'mode':str(h.getMotorControlMode(a)),'sdk':h.getSDKVersion()}
 fw=h.getFirmwareVersion(a)
 result['firmware']={k:str(getattr(fw,k)) for k in dir(fw) if not k.startswith('_') and not callable(getattr(fw,k))}
 result['faults']=[{'motor':getattr(f,'motor_ID',None),'finger':getattr(f,'finger_name',None),'error':str(f.error_code)} for f in h.getFaultCode(a)]
 p=h.getMotorPosition(a);result['positions']=[getattr(p,f'motor{i}_pos') for i in range(1,8)]
 result['magnetic']=list(h.getJointMagneticEncoder(a))
 result['init_related_interfaces']=[n for n in dir(h) if 'init' in n.lower() or 'online' in n.lower()]
 result['motor_commands_sent']=0
 print(json.dumps(result,ensure_ascii=False),flush=True)
finally:h.disconnectUsb()
