"""GMH15 identification/status only. No initialization or actuator writes."""
import argparse
import json
from pathlib import Path

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port')
    parser.add_argument('--expected-sn', default='H15L49G00062')
    parser.add_argument('--sdk-check', action='store_true')
    args = parser.parse_args()
    from pxdex.dh15 import DexH15Control
    hand = DexH15Control()
    print(json.dumps({'sdk_version': hand.getSDKVersion()}, ensure_ascii=False), flush=True)
    if args.sdk_check:
        return 0
    ports = sorted(str(p) for p in Path('/dev').glob('ttyACM*'))
    port = args.port
    if port is None:
        if len(ports) != 1:
            print(json.dumps({'connected': False, 'usb_serial_ports': ports,
                              'reason': 'Expected exactly one USB ACM port; specify --port explicitly otherwise.'}), flush=True)
            return 2
        port = ports[0]
    if not Path(port).exists():
        print(json.dumps({'connected': False, 'reason': 'Serial device absent', 'port': port}), flush=True)
        return 2
    connected = False
    try:
        connected, address = hand.openUsbDevice(port)
        if not connected:
            print(json.dumps({'connected': False, 'port': port}), flush=True)
            return 3
        sn = hand.getDeviceSN(address)
        print(json.dumps({'connected': True, 'port': port, 'address': address, 'sn': sn}), flush=True)
        if sn != args.expected_sn:
            print('SN mismatch: stopping before further queries.', flush=True)
            return 4
        hand_type = str(hand.getHandType(address))
        if hand_type != 'HandType.LEFT_HAND':
            print(json.dumps({'reason': 'Expected left hand', 'hand_type': hand_type}), flush=True)
            return 5
        result = {
            'hand_type': hand_type,
            'online': hand.isModbusDeviceConnected(address),
            'hand_status': hand.checkHandStatus(address),
            'motor_enabled': hand.isMotorEnabled(address),
            'control_mode': str(hand.getMotorControlMode(address)),
            'fault_codes': [str(f) for f in hand.getFaultCode(address)],
        }
        print(json.dumps(result, ensure_ascii=False), flush=True)
        return 0
    finally:
        if connected:
            hand.disconnectUsb()

if __name__ == '__main__':
    raise SystemExit(main())
