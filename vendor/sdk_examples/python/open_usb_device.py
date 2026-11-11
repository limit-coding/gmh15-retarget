"""USB direct connection example.

Steps:
    1. Create the serial control instance.
    2. Open the USB serial port.
    3. Initialize motor position.
    4. Query online status.
    5. Close the USB serial port.
"""

import argparse
import sys
from pxdex.dh15 import DexH15Control


# USB direct connection usage reference example
if __name__ == "__main__":
    """
    Steps:
        1. Open USB serial port [required]
        2. Initialize motor position [required for position control, call once after power-on and connection]
        3. Other operations, e.g., query dexterous hand online status [optional]
        4. Close USB serial port [required]
    """
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/ttyACM0", help="USB serial port, default: /dev/ttyACM0")
    args = parser.parse_args()

    # 1. Create serial control instance
    hand_control = DexH15Control()

    # 2. Open USB serial port
    ret, slave_address = hand_control.openUsbDevice(args.port)
    if not ret:
        print("Please check if the USB serial device is connected")
        sys.exit(1)
    print("Open USB serial port success")

    hand_type = hand_control.getHandType(slave_address)
    hand_type_name = str(hand_type).split(".")[-1]
    print(f"Modbus device address: 0x{slave_address:02x}")
    print(f"Hand type: {hand_type_name}")
    if hand_type_name == "NONE_HAND":
        hand_control.disconnectUsb()
        sys.exit(1)

    # 3. Initialize motor position (can skip if already called after power-on)
    if hand_control.initMotorPosition(slave_address) != 1:
        print("Init motor position failed")
        hand_control.disconnectUsb()
        sys.exit(1)
    print("Init motor position success")

    # 4. Query dexterous hand online status
    is_connected = hand_control.isModbusDeviceConnected(slave_address)
    print(f"Dexterous hand {'online' if is_connected else 'offline'}")

    # 5. Close USB serial port
    result = hand_control.disconnectUsb()
    print(f"Disconnect: {'success' if result else 'failed'}")
