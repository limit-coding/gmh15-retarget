"""Force grasp demo for normal or tangential force control.

Steps:
    1. Connect to a Modbus slave automatically or by address.
    2. Initialize motor position and switch to position mode.
    3. Set normal or tangential force-control parameters.
    4. Compute and execute target joint-angle commands.
    5. Return to zero, disable motors, and close the Modbus serial port.
"""

import argparse
import sys
import time

from pxdex.dh15 import ControlMode, Dex15MotorPosition, DexH15Control, ForceCtlParam, TanForceCtlParam


CONTROL_HZ = 20
CONTROL_DURATION_SECONDS = 5
FORCE_READY_ANGLE = [900, 270, 960, 300, 1000, 330, 1000, 360, 500, 240, 180]
RETURN_ZERO_WAIT_SECONDS = 2.0


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


def make_common_force_param():
    param = ForceCtlParam()
    param.impedance_factor = [30] * 11
    param.reference_force_factor = [3] * 11
    param.enable_flag = [True, True, True, True, False, False]
    return param


def make_tangential_force_param():
    param = TanForceCtlParam()
    param.tangential_force_gain = 2.0
    param.impedance_factor = [30] * 11
    param.reference_force_factor = [3] * 11
    param.enable_flag = [True, True, True, True, False, False]
    return param


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="/dev/ttyUSB0", help="Modbus serial port, default: /dev/ttyUSB0")
    parser.add_argument(
        "--addr",
        type=lambda value: int(value, 0),
        default=None,
        help="Optional Modbus slave address. If omitted, connectModbusAuto is used.",
    )
    parser.add_argument("--mode", choices=["normal", "tangential"], default="normal")
    args = parser.parse_args()

    # Create hand control instance
    hand_control = DexH15Control()
    # Connect to the selected hand. If --addr is omitted, connect to the first slave found.
    slave_address = connect_hand(hand_control, args.port, args.addr)

    # Query the hand type
    hand_type = hand_control.getHandType(slave_address)
    print(f"Modbus device address: 0x{slave_address:02x}")
    print(f"Hand type: {str(hand_type).split('.')[-1]}")

    if not prepare_position_mode(hand_control, slave_address):
        hand_control.disconnectModbus()
        sys.exit(1)

    print("Move to force-control ready pose")
    # Move the hand to the initial pose for force control
    if hand_control.setJointPositionsAngle(slave_address, FORCE_READY_ANGLE) != 1:
        print("Move to force-control ready pose failed")
        hand_control.disconnectModbus()
        sys.exit(1)
    time.sleep(1.5)

    original_angle = FORCE_READY_ANGLE
    if args.mode == "normal":
        # Set normal force-control parameters
        if hand_control.setCommonForceCtlParam(slave_address, original_angle, make_common_force_param()) != 1:
            print("Set normal force-control parameters failed")
            hand_control.disconnectModbus()
            sys.exit(1)
    else:
        # Set tangential force-control parameters
        if hand_control.setTangentialForceCtlParam(slave_address, original_angle, make_tangential_force_param()) != 1:
            print("Set tangential force-control parameters failed")
            hand_control.disconnectModbus()
            sys.exit(1)

    time.sleep(1)
    start_time = time.time()
    try:
        while time.time() - start_time < CONTROL_DURATION_SECONDS:
            if args.mode == "normal":
                # Compute the next normal force-control target command
                ret, command = hand_control.computeCommonForceTargetCommand(slave_address)
            else:
                # Compute the next tangential force-control target command
                ret, command = hand_control.computeTangentialForceTargetCommand(slave_address)
            if not ret:
                print("Compute force-control target command failed")
                break
            # Send the computed joint-angle command
            if hand_control.setJointPositionsAngle(slave_address, command) != 1:
                print("Set force-control target command failed")
                break
            time.sleep(1.0 / CONTROL_HZ)
    except KeyboardInterrupt:
        print("\nUser interrupted")
    finally:
        # Return motor-side target positions to zero
        if hand_control.setMotorTargetPosition(slave_address, Dex15MotorPosition()) != 1:
            print("Return motors to zero failed")
        else:
            print("Return motors to zero")
            time.sleep(RETURN_ZERO_WAIT_SECONDS)
        # Disable motors before disconnecting
        print(f"Disable motors: {'success' if hand_control.disableMotor(slave_address) else 'failed'}")
        # Close the Modbus serial port
        result = hand_control.disconnectModbus()
        print(f"Disconnect: {'success' if result else 'failed'}")
