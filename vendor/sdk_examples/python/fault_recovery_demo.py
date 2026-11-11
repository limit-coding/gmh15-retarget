"""Fault recovery workflow demo.

Steps:
    1. Connect to a Modbus slave automatically or by address.
    2. Read hand status and motor fault codes.
    3. Clear motor fault codes.
    4. Recheck hand status and fault codes.
    5. Close the Modbus serial port.
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
    if address is None:
        # Search and connect to the first available Modbus slave
        connect_ret, slave_address = hand_control.connectModbusAuto(port)
        if not connect_ret:
            print("Please check if the dexterous hand is connected to the Modbus serial port")
            sys.exit(1)
        return slave_address

    # Open the Modbus serial port
    if not hand_control.openModbusDevice(port, 4000000):
        print("Please check if the Modbus serial device is connected")
        sys.exit(1)
    # Initialize the specified Modbus slave
    if hand_control.initModbusDevice(address) != 1:
        print("Please check if the dexterous hand is connected to the Modbus serial device")
        hand_control.disconnectModbus()
        sys.exit(1)
    return address


def print_fault_codes(fault_codes):
    if not fault_codes:
        print("Fault code: none")
        return
    for fault in fault_codes:
        print(f"Fault: finger={fault.finger_name}, motor={fault.motor_ID}, code={fault.error_code}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/ttyUSB0", help="Modbus serial port, default: /dev/ttyUSB0")
    parser.add_argument(
        "--addr",
        type=lambda value: int(value, 0),
        default=None,
        help="Optional Modbus slave address. If omitted, connectModbusAuto is used.",
    )
    args = parser.parse_args()

    # Create hand control instance
    hand_control = DexH15Control()
    slave_address = connect_hand(hand_control, args.port, args.addr)

    # Query the hand type
    hand_type = hand_control.getHandType(slave_address)
    print(f"Modbus device address: 0x{slave_address:02x}")
    print(f"Hand type: {str(hand_type).split('.')[-1]}")

    # Read hand status and fault codes before clearing
    print(f"Before clear, hand status: {hand_status_name(hand_control.checkHandStatus(slave_address))}")
    print_fault_codes(hand_control.getFaultCode(slave_address))

    # Clear current motor fault codes
    cleared = hand_control.clearFaultCode(slave_address)
    print(f"Clear fault codes: {'success' if cleared else 'failed'}")

    # Recheck hand status and fault codes after clearing
    print(f"After clear, hand status: {hand_status_name(hand_control.checkHandStatus(slave_address))}")
    print_fault_codes(hand_control.getFaultCode(slave_address))

    # Close the Modbus serial port
    result = hand_control.disconnectModbus()
    print(f"Disconnect: {'success' if result else 'failed'}")
