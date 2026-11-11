"""Touch-triggered finger bend demo.

Steps:
    1. Connect to a Modbus slave automatically or by address.
    2. Open the hand, calibrate tactile sensors, and record the baseline.
    3. Touch or swipe any tactile area on index/middle/ring/pinky in any order.
    4. Bend the touched finger to the fist posture; thumb responds only after the other four.
    5. Return to open posture, disable motors, and close the Modbus serial port.
"""

import argparse
import math
import sys
import time
from dataclasses import dataclass

from pxdex.dh15 import ControlMode, Dex15MotorCurrent, Dex15MotorPosition, Dex15MotorSpeed, DexH15Control, FingerType


DEFAULT_DURATION_SECONDS = 25
DEFAULT_HZ = 12
DEFAULT_TOUCH_THRESHOLD = 1
DEBOUNCE_S = 0.25
COMPLETE_HOLD_S = 1.6
THUMB_READY_SETTLE_S = 0.8
RETURN_ZERO_WAIT_S = 2.0


@dataclass
class FingerSpec:
    finger_type: object
    name: str
    joint_indices: list
    bent_values: list
    baseline: object = None
    baseline_detail: list = None
    bent: bool = False
    available: bool = True


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


def motor_speed(values):
    speed = Dex15MotorSpeed()
    (
        speed.motor1_speed,
        speed.motor2_speed,
        speed.motor3_speed,
        speed.motor4_speed,
        speed.motor5_speed,
        speed.motor6_speed,
        speed.motor7_speed,
    ) = values
    return speed


def motor_current(values):
    current = Dex15MotorCurrent()
    (
        current.motor1_current,
        current.motor2_current,
        current.motor3_current,
        current.motor4_current,
        current.motor5_current,
        current.motor6_current,
        current.motor7_current,
    ) = values
    return current


def has_non_zero_speed(speed):
    return any(
        (
            speed.motor1_speed,
            speed.motor2_speed,
            speed.motor3_speed,
            speed.motor4_speed,
            speed.motor5_speed,
            speed.motor6_speed,
            speed.motor7_speed,
        )
    )


def current_values(current):
    return (
        current.motor1_current,
        current.motor2_current,
        current.motor3_current,
        current.motor4_current,
        current.motor5_current,
        current.motor6_current,
        current.motor7_current,
    )


def has_non_zero_current(current):
    return any(current_values(current))


def is_valid_current_limit(current):
    return all(0 <= value <= 1000 for value in current_values(current))


def set_motor_max_current_no_throw(hand_control, slave_address, current, error_context):
    try:
        # Set the motor max current limit
        return hand_control.setMotorMaxCurrent(slave_address, current)
    except RuntimeError as error:
        print(f"{error_context}: {error}")
        return False


def motion_speed():
    return motor_speed((650, 650, 650, 650, 420, 420, 420))


def return_speed():
    return motor_speed((700, 700, 700, 740, 700, 700, 700))


def motion_current_limit():
    return motor_current((1000, 1000, 1000, 1000, 1000, 1000, 1000))


def return_current_limit():
    return motor_current((1000, 1000, 1000, 1000, 1000, 1000, 1000))


def open_joint_target():
    return [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]


def thumb_ready_joint_target():
    target = open_joint_target()
    target[8] = 250
    target[9] = 0
    target[10] = 0
    return target


def make_finger_specs():
    return [
        FingerSpec(FingerType.INDEX_FINGER, "index", [0, 1], [1000, 1000]),
        FingerSpec(FingerType.MIDDLE_FINGER, "middle", [2, 3], [1000, 1000]),
        FingerSpec(FingerType.RING_FINGER, "ring", [4, 5], [1000, 1000]),
        FingerSpec(FingerType.PINKY_FINGER, "pinky", [6, 7], [1000, 1000]),
        FingerSpec(FingerType.THUMB_FINGER, "thumb", [8, 9, 10], [500, 900, 900]),
    ]


def is_thumb(finger):
    return finger.finger_type == FingerType.THUMB_FINGER


def regular_bent_count(fingers):
    return sum(1 for finger in fingers if not is_thumb(finger) and finger.bent)


def all_fingers_bent(fingers):
    return all(finger.bent for finger in fingers)


def read_finger_touch(hand_control, slave_address, finger_type):
    try:
        # Read resultant force, distributed force, and temperature for one finger
        force, detail, _ = hand_control.getFingerTactileData(slave_address, finger_type)
        return force, detail
    except RuntimeError as error:
        print(f"Read finger tactile data failed: {error}")
        return None


def force_point_score(force, baseline):
    normal_delta = abs(force.z - baseline.z)
    tangential_delta = round(math.hypot(force.x - baseline.x, force.y - baseline.y))
    return max(normal_delta, tangential_delta)


def touch_score(force, detail, finger):
    score = force_point_score(force, finger.baseline)
    baseline_detail = finger.baseline_detail or []
    for index, point in enumerate(detail):
        baseline = baseline_detail[index] if index < len(baseline_detail) else None
        if baseline is None:
            normal_delta = abs(point.z)
            tangential_delta = round(math.hypot(point.x, point.y))
            score = max(score, normal_delta, tangential_delta)
        else:
            score = max(score, force_point_score(point, baseline))
    return score


def apply_finger_target(finger, target):
    for index, value in zip(finger.joint_indices, finger.bent_values):
        target[index] = value


def apply_joint_target(hand_control, slave_address, target, step_name):
    # Send joint-angle target command
    if hand_control.setJointPositionsAngle(slave_address, target) != 1:
        print(f"Set {step_name} failed")
        return False
    return True


def return_to_open(hand_control, slave_address):
    # Set return speed before moving back to open posture
    hand_control.setMotorMaxVelocity(slave_address, return_speed())
    set_motor_max_current_no_throw(
        hand_control, slave_address, return_current_limit(), "Set return current limit failed"
    )

    # Return all motors to the homing zero position before disabling motors.
    if hand_control.setMotorTargetPosition(slave_address, Dex15MotorPosition()) != 1:
        print("Return motors to zero position failed")
        return
    time.sleep(RETURN_ZERO_WAIT_S)


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

    # Save current speed and current-limit configuration
    original_speed = hand_control.getMotorMaxVelocityConfig(slave_address)
    original_current = hand_control.getMotorMaxCurrentConfig(slave_address)
    should_restore_speed = has_non_zero_speed(original_speed)
    original_current_non_zero = has_non_zero_current(original_current)
    should_restore_current = original_current_non_zero and is_valid_current_limit(original_current)

    target = open_joint_target()

    try:
        # Set motion speed for touch-triggered bending
        if not hand_control.setMotorMaxVelocity(slave_address, motion_speed()):
            print("Set motion speed failed")
            sys.exit(1)
        if not set_motor_max_current_no_throw(
            hand_control, slave_address, motion_current_limit(), "Set motion current limit failed"
        ):
            sys.exit(1)

        target = thumb_ready_joint_target()
        # Move the thumb to a ready posture before tactile triggering starts.
        apply_joint_target(hand_control, slave_address, target, "thumb ready target")
        time.sleep(THUMB_READY_SETTLE_S)

        # Calibrate tactile sensors before recording the touch baseline
        if not hand_control.calibrateSensor(slave_address):
            print("Sensor calibration failed, continue with current baseline")
        else:
            print("Sensor calibration success")

        fingers = make_finger_specs()
        for finger in fingers:
            # Read the baseline tactile value for this finger
            reading = read_finger_touch(hand_control, slave_address, finger.finger_type)
            finger.available = reading is not None
            if reading is not None:
                finger.baseline, finger.baseline_detail = reading
            if not finger.available:
                print(f"{finger.name} tactile unavailable, skip this finger")

        print("Touch or swipe any tactile area on index/middle/ring/pinky in any order.")
        print("Touch thumb after those four. Normal and tangential force changes are both accepted.")
        print(f"Duration: {DEFAULT_DURATION_SECONDS}s")

        thumb_wait_notified = False
        deadline = time.monotonic() + DEFAULT_DURATION_SECONDS
        period_s = 1.0 / DEFAULT_HZ
        while time.monotonic() < deadline and not all_fingers_bent(fingers):
            for finger in fingers:
                if finger.bent or not finger.available:
                    continue

                # Read current tactile value for this finger
                reading = read_finger_touch(hand_control, slave_address, finger.finger_type)
                if reading is None:
                    finger.available = False
                    continue

                force, detail = reading
                score = touch_score(force, detail, finger)
                if score < DEFAULT_TOUCH_THRESHOLD:
                    continue

                if is_thumb(finger) and regular_bent_count(fingers) < 4:
                    if not thumb_wait_notified:
                        print("Thumb touched, waiting until index/middle/ring/pinky are triggered")
                        thumb_wait_notified = True
                    continue

                apply_finger_target(finger, target)
                if not apply_joint_target(hand_control, slave_address, target, f"{finger.name} bend target"):
                    continue

                finger.bent = True
                thumb_wait_notified = False
                print(f"Triggered {finger.name}, touch score: {score}")
                time.sleep(DEBOUNCE_S)

            time.sleep(period_s)

        if all_fingers_bent(fingers):
            print("All fingers bend completed, return to zero posture")
            time.sleep(COMPLETE_HOLD_S)
        else:
            print("Demo timeout, return to zero posture")

    except KeyboardInterrupt:
        print("\nUser interrupted")
    finally:
        return_to_open(hand_control, slave_address)
        if should_restore_current:
            set_motor_max_current_no_throw(
                hand_control, slave_address, original_current, "Restore motor current limit config failed"
            )
        # Restore the original speed configuration
        if should_restore_speed and not hand_control.setMotorMaxVelocity(slave_address, original_speed):
            print("Restore motor speed config failed")
        # Disable motors before disconnecting
        print(f"Disable motors: {'success' if hand_control.disableMotor(slave_address) else 'failed'}")
        # Close the Modbus serial port
        result = hand_control.disconnectModbus()
        print(f"Disconnect: {'success' if result else 'failed'}")
