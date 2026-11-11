"""Scan Modbus devices on the serial port.

Steps:
    1. Open the Modbus serial port.
    2. Scan slave addresses on the port.
    3. Query hand type and online status for each device.
    4. Close the Modbus serial port.
"""

import argparse
import sys

from pxdex.dh15 import DexH15Control


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/ttyUSB0", help="Modbus serial port, default: /dev/ttyUSB0")
    args = parser.parse_args()

    hand_control = DexH15Control()
    # 1. Open the Modbus serial port before scanning
    if not hand_control.openModbusDevice(args.port, 4000000):
        print("Please check if the Modbus serial device is connected")
        sys.exit(1)

    # 2. Scan all Modbus slave addresses on this port
    address_list = hand_control.scanModbusDevices(args.port)
    if not address_list:
        print(f"No Modbus devices found on {args.port}")
        hand_control.disconnectModbus()
        sys.exit(0)

    print("Found Modbus devices:")
    # 3. Query hand type and online status for each discovered address
    for slave_address in address_list:
        hand_type = hand_control.getHandType(slave_address)
        is_connected = hand_control.isModbusDeviceConnected(slave_address)
        print(
            f"  address 0x{slave_address:02x}, "
            f"{str(hand_type).split('.')[-1]}, "
            f"{'online' if is_connected else 'offline'}"
        )

    # 4. Close Modbus serial port
    ret = hand_control.disconnectModbus()
    print(f"Disconnect: {'success' if ret else 'failed'}")
