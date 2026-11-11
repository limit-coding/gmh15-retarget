"""Comprehensive bringup check.

Steps:
    1. Open Modbus serial port and scan/select a slave.
    2. Query hand type, online status, and hand status.
    3. Query SDK version, Modbus baudrate, motor state, and fault codes.
    4. Close the Modbus serial port.
"""

import argparse
import sys

from pxdex.dh15 import DexH15Control


def hand_status_name(status):
    if status == 1:
        return "normal operation"
    if status == 2:
        return "fault-tolerant operation"
    return "non-operational"


def connect_hand(hand_control, port, address):
    # Open the Modbus serial port
    if not hand_control.openModbusDevice(port, 4000000):
        print("Please check if the Modbus serial device is connected")
        sys.exit(1)
    print("Open Modbus serial port success")

    if address is None:
        # Scan available Modbus slave addresses on the selected port
        address_list = hand_control.scanModbusDevices(port)
        if not address_list:
            print(f"No Modbus devices found on {port}")
            hand_control.disconnectModbus()
            sys.exit(1)

        print("Found Modbus devices:")
        for slave_address in address_list:
            print(f"  address 0x{slave_address:02x}")
        address = address_list[0]
        print(f"Use first Modbus device: 0x{address:02x}")

    # Initialize the selected Modbus slave
    if hand_control.initModbusDevice(address) != 1:
        print("Please check if the dexterous hand is connected to the Modbus serial device")
        hand_control.disconnectModbus()
        sys.exit(1)
    print("Init Modbus slave success")
    return address


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/ttyUSB0", help="Modbus serial port, default: /dev/ttyUSB0")
    parser.add_argument(
        "--addr",
        type=lambda value: int(value, 0),
        default=None,
        help="Optional Modbus slave address. If omitted, the first scanned device is used.",
    )
    args = parser.parse_args()

    # Create hand control instance
    hand_control = DexH15Control()
    slave_address = connect_hand(hand_control, args.port, args.addr)

    print("\nBasic status")
    # Query basic hand status through SDK interfaces
    hand_type = hand_control.getHandType(slave_address)
    print(f"Modbus device address: 0x{slave_address:02x}")
    print(f"Hand type: {str(hand_type).split('.')[-1]}")
    print(f"Online: {'yes' if hand_control.isModbusDeviceConnected(slave_address) else 'no'}")
    print(f"Hand status: {hand_status_name(hand_control.checkHandStatus(slave_address))}")
    print(f"SDK version: {hand_control.getSDKVersion()}")
    print(f"Modbus baudrate: {hand_control.getModbusBaudrate(slave_address)}")

    print("\nRuntime state")
    # Query motor enable state and current control mode
    print(f"Motor enabled: {'yes' if hand_control.isMotorEnabled(slave_address) else 'no'}")
    print(f"Control mode: {hand_control.getMotorControlMode(slave_address)}")

    # Read current motor fault codes
    fault_codes = hand_control.getFaultCode(slave_address)
    if not fault_codes:
        print("Fault code: none")
    for fault in fault_codes:
        print(f"Fault: finger={fault.finger_name}, motor={fault.motor_ID}, code={fault.error_code}")

    # Close the Modbus serial port
    result = hand_control.disconnectModbus()
    print(f"Disconnect: {'success' if result else 'failed'}")
