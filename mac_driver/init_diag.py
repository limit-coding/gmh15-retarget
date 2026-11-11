"""Operator-requested position-initialization diagnostic.

Sequence: identity/status read -> clearFaultCode (vendor recovery API) -> initMotorPosition once
-> disableMotor in all cases. Never enables position control or sends motor targets.
Per-motor zero state (register 0x2120) is visible in the Mac bridge trace (GMH15_USB_TRACE).
"""
import json
import sys
import time
from pxdex.dh15 import DexH15Control

EXPECTED_SN = sys.argv[1] if len(sys.argv) > 1 else None
CLEAR = '--no-clear' not in sys.argv

hand = DexH15Control()
ok, address = hand.openUsbDevice('/tmp/gmh15-serial')
if not ok:
    raise SystemExit('USB connection failed')


def report(event, **fields):
    print('GMH15_DIAG ' + json.dumps(dict(event=event, t=round(time.monotonic(), 3), **fields)), flush=True)


try:
    sn = hand.getDeviceSN(address)
    if EXPECTED_SN and sn != EXPECTED_SN:
        raise SystemExit(f'Unexpected SN {sn}')
    if hand.isMotorEnabled(address):
        raise SystemExit('Motors already enabled; refusing')
    faults = [str(f.error_code) for f in hand.getFaultCode(address)]
    report('pre', sn=sn, status=hand.checkHandStatus(address), faults=faults,
           mode=str(hand.getMotorControlMode(address)))
    if CLEAR:
        report('clear_fault', ok=bool(hand.clearFaultCode(address)),
               faults_after=[str(f.error_code) for f in hand.getFaultCode(address)])
        time.sleep(.3)
    started = time.monotonic()
    result = int(hand.initMotorPosition(address))
    p = hand.getMotorPosition(address)
    report('init', result=result, duration_ms=round((time.monotonic() - started) * 1000),
           motors=[getattr(p, f'motor{i}_pos') for i in range(1, 8)],
           faults=[str(f.error_code) for f in hand.getFaultCode(address)])
finally:
    try:
        hand.disableMotor(address)
        report('disabled', motor_enabled=bool(hand.isMotorEnabled(address)))
    finally:
        hand.disconnectUsb()
