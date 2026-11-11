"""Modbus auto-connection example.

Steps:
    1. Create the serial control instance.
    2. Automatically connect to the first slave on the Modbus serial port.
    3. Query hand type and online status.
    4. Close the Modbus serial port.
"""

import argparse
import sys

from pxdex.dh15 import DexH15Control


# Auto-connect usage reference example
if __name__ == "__main__":
    """
    Steps:
        1. Create serial control instance [required]
        2. Auto-connect device [required]
        3. Other operations, e.g., connection check [optional]
        4. Close Modbus serial port [required]
    """
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/ttyUSB0", help="Modbus serial port, default: /dev/ttyUSB0")
    args = parser.parse_args()

    # 1. Create serial control instance
    hand_control = DexH15Control()

    # 2. Auto-connect device
    connect_ret, slave_address = hand_control.connectModbusAuto(args.port)
    print(f"Auto connect: {'success' if connect_ret else 'failed'}")
    if not connect_ret:
        sys.exit(1)

    hand_type = hand_control.getHandType(slave_address)
    hand_type_name = str(hand_type).split(".")[-1]
    print(f"Modbus device address: 0x{slave_address:02x}")
    print(f"Hand type: {hand_type_name}")
    if hand_type_name == "NONE_HAND":
        hand_control.disconnectModbus()
        sys.exit(1)

    # 3. Other operations, e.g., connection check
    is_connect = hand_control.isModbusDeviceConnected(slave_address)
    print(f"Connection status: {'connected' if is_connect else 'disconnected'}")

    # 4. Close Modbus serial port
    ret = hand_control.disconnectModbus()
    print(f"Disconnect: {'success' if ret else 'failed'}")
