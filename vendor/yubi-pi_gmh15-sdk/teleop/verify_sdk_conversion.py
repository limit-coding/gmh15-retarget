"""Pure SDK angle conversion; never constructs a device controller."""
import json
from pxdex.dh15 import DexH15Kinematic
kin=DexH15Kinematic()
real=[10.,0.,0.,0.,0.,0.,0.,0.,54.8,0.,0.]
ret,normalized=kin.calculateNormalizeAngle(real,'E ')
assert ret==1 and len(normalized)==11,(ret,len(normalized))
ret2,roundtrip=kin.calculateRealAngle(normalized,'E ')
assert ret2==1 and max(abs(a-b) for a,b in zip(real,roundtrip))<.2
print(json.dumps({'sdk_conversion_success':True,'input_real_degrees':real,'output_normalized':list(normalized),
                  'roundtrip_error_degrees':max(abs(a-b) for a,b in zip(real,roundtrip)),
                  'serial_device_opened':False,'motor_commands_sent':0}))
