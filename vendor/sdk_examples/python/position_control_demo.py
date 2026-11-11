"""Position-control demo.

Steps:
    1. Connect to a Modbus slave automatically or by address.
    2. Initialize motor position and switch to position mode.
    3. Send a small target position.
    4. Return to zero and disable motors.
    5. Close the Modbus serial port.
"""

import argparse
import sys
import time

from pxdex.dh15 import ControlMode, Dex15MotorPosition, DexH15Control


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


def prepare_position_mode(hand_control, slave_address):
    # Initialize motor position before enabling control
    if hand_control.initMotorPosition(slave_address) != 1:
        print("Init motor position failed")
        return False
    # Check and disable motors before switching control mode
    if hand_control.isMotorEnabled(slave_address):
        if not hand_control.disableMotor(slave_address):
            print("Disable motor failed, cannot set position mode")
            return False
    # Switch to position control mode
    if not hand_control.setMotorControlMode(slave_address, ControlMode.POSITION_CONTROL_MODE):
        print("Set position mode failed")
        return False
    # Enable motors after the control mode is ready
    if not hand_control.enableMotor(slave_address):
        print("Enable motor failed")
        return False
    return True


def motor_position(values):
    position = Dex15MotorPosition()
    (
        position.motor1_pos,
        position.motor2_pos,
        position.motor3_pos,
        position.motor4_pos,
        position.motor5_pos,
        position.motor6_pos,
        position.motor7_pos,
    ) = values
    return position


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

    if not prepare_position_mode(hand_control, slave_address):
        hand_control.disconnectModbus()
        sys.exit(1)

    demo_positions = motor_position((2000, 2000, 2000, 2000, 800, 1200, 1200))
    # Send target motor positions
    if hand_control.setMotorTargetPosition(slave_address, demo_positions) != 1:
        print("Set demo target position failed")
    else:
        print("Set demo target position success")
    time.sleep(1)

    zero_positions = Dex15MotorPosition()
    # Return motor-side target positions to zero
    if hand_control.setMotorTargetPosition(slave_address, zero_positions) != 1:
        print("Return motors to zero failed")
    else:
        print("Return motors to zero success")
    time.sleep(0.5)

    # Disable motors before disconnecting
    print(f"Disable motors: {'success' if hand_control.disableMotor(slave_address) else 'failed'}")
    # Close the Modbus serial port
    result = hand_control.disconnectModbus()
    print(f"Disconnect: {'success' if result else 'failed'}")
