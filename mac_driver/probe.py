"""One-shot identity/status query. Never initialize, enable, or set targets."""
import json
import sys
from pxdex.dh15 import DexH15Control, DexH15Kinematic

hand = DexH15Control()
ok, address = hand.openUsbDevice('/tmp/gmh15-serial')
if not ok:
    raise SystemExit('USB connection failed')
try:
    hw = hand.getHardwareVersion(address).mainboard_hardware_version
    side = hand.getHandType(address)
    positions = hand.getMotorPosition(address)
    motors = [getattr(positions, f'motor{i}_pos') for i in range(1, 8)]
    encoders = list(hand.getJointMagneticEncoder(address))
    ret, degrees = DexH15Kinematic().forwardKinematicParse(side, motors, encoders, str(hw), True)
    result = dict(sn=hand.getDeviceSN(address), hand_type=str(side), hardware_version=str(hw),
                  motor_enabled=bool(hand.isMotorEnabled(address)), status=hand.checkHandStatus(address),
                  faults=[str(f.error_code) for f in hand.getFaultCode(address)],
                  motors=motors, encoders=encoders, joint_result=ret, joints=list(degrees))
    print('GMH15_PROBE ' + json.dumps(result), flush=True)
finally:
    hand.disconnectUsb()
