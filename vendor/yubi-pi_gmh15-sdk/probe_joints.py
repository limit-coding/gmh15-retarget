import json
from pxdex.dh15 import DexH15Control, DexH15Kinematic
h=DexH15Control();ok,a=h.openUsbDevice('/dev/ttyACM0')
if not ok:raise SystemExit('USB failed')
try:
 print('SN',h.getDeviceSN(a),flush=True)
 p=h.getMotorPosition(a)
 print('MOTORS', [getattr(p,f'motor{i}_pos') for i in range(1,8)],flush=True)
 for all_joints in (False,True):
  print('JOINTS',all_joints,h.getJointPositionsAngle(a,is_all_joint=all_joints),flush=True)
 print('HARDWARE',h.getHardwareVersion(a).mainboard_hardware_version,flush=True)
 enc=h.getJointMagneticEncoder(a)
 print('ENCODERS',list(enc),flush=True)
 kin=DexH15Kinematic()
 print('FK15',kin.forwardKinematicParse(h.getHandType(a),[getattr(p,f'motor{i}_pos') for i in range(1,8)],enc,h.getHardwareVersion(a).mainboard_hardware_version,True),flush=True)
 print('STATUS',h.checkHandStatus(a),h.isMotorEnabled(a),flush=True)
except Exception as e:
 print(type(e).__name__,str(e),flush=True)
finally:h.disconnectUsb()
