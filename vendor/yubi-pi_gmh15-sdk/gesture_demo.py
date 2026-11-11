"""Run gesture action example.

Steps:
    1. Connect to the dexterous hand.
    2. Initialize motor position.
    3. Set position control mode and enable motors.
    4. Run the selected gesture sequence.
    5. Disable motors and close the Modbus serial port.
"""

import argparse
import sys
import time

from pxdex.dh15 import ControlMode, Dex15MotorSpeed, DexH15Control


WAVE_REPEATS = 5
WAVE_HOLD_SECONDS = 0.15
WAVE_FINGER_STEPS = [
    ("index finger", (650, 300, 0, 0, 0, 0, 0, 0, 0, 0, 0)),
    ("middle finger", (0, 0, 650, 300, 0, 0, 0, 0, 0, 0, 0)),
    ("ring finger", (0, 0, 0, 0, 650, 300, 0, 0, 0, 0, 0)),
    ("pinky finger", (0, 0, 0, 0, 0, 0, 650, 300, 0, 0, 0)),
]


IMPLEMENTED_GESTURES = {1, 2, 3, 4, 5, 6, 7, 8}
ALL_GESTURE_IDS = tuple(sorted(IMPLEMENTED_GESTURES))
TEMPORARY_SPEED_GESTURES = {1, 2, 3, 5, 6, 7, 8}

GESTURE_HELP_LINES = (
    "  1: fist stretch",
    "  2: sequential 4-finger close",
    "  3: ok gesture",
    "  4: sequential wave (index->middle->ring->pinky repeat)",
    "  5: peace sign",
    "  6: cup initial grasp",
    "  7: middle-ring pinch",
    "  8: sequential curl (middle->ring->pinky, thumb upright)",
)


BASE_GESTURES = {
    1: [
        ("open", (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0), 0.3),
        ("close fist together", (1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 420, 900, 900), 1.8),
    ],
    2: [
        ("open", (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0), 0.4),
        ("close index finger", (1000, 1000, 0, 0, 0, 0, 0, 0, 0, 0, 0), 0.7),
        ("close middle finger", (1000, 1000, 1000, 1000, 0, 0, 0, 0, 0, 0, 0), 0.7),
        ("close ring finger", (1000, 1000, 1000, 1000, 1000, 1000, 0, 0, 0, 0, 0), 0.7),
        ("close pinky finger", (1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 0, 0, 0), 0.7),
        ("thumb outer arc", (1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 500, 320, 150), 1.15),
        (
            "thumb approach middle fingertip",
            (1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 500, 650, 600),
            0.75,
        ),
        ("close thumb", (1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 420, 900, 900), 1.8),
    ],
    3: [
        ("open", (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0), 0.4),
        ("ok gesture", (1000, 1000, 350, 350, 220, 220, 120, 120, 320, 700, 700), 2.5),
    ],
    5: [
        ("open", (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0), 0.35),
        ("peace sign", (0, 0, 0, 0, 1000, 1000, 1000, 1000, 500, 880, 520), 2.5),
    ],
    6: [
        ("open", (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0), 0.5),
        ("cup initial grasp", (900, 270, 960, 300, 1000, 330, 1000, 360, 500, 240, 180), 2.5),
    ],
    7: [
        ("open", (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0), 0.35),
        ("pinch ready", (180, 60, 500, 260, 420, 200, 260, 100, 360, 420, 300), 0.5),
        ("middle-ring pinch", (820, 560, 900, 700, 720, 500, 720, 500, 420, 620, 420), 1.0),
    ],
    8: [
        ("open", (0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0), 0.35),
        ("simultaneous palm-aligned pose", (0, 0, 350, 0, 700, 0, 1000, 0, 250, 750, 200), 2.5),
    ],
}


def print_gesture_help():
    print("Implemented gestures:")
    for line in GESTURE_HELP_LINES:
        print(line)


def keep_thumb_posture(values, thumb_values=None):
    values = list(values)
    if thumb_values is not None:
        values[8:11] = thumb_values
    return tuple(values)


def build_gesture(gesture_id, thumb_values=None):
    if gesture_id == 1:
        return BASE_GESTURES[gesture_id]

    if gesture_id == 2:
        return [
            (name, keep_thumb_posture(values, thumb_values), hold_seconds)
            if name != "close thumb"
            else (name, values, hold_seconds)
            for name, values, hold_seconds in BASE_GESTURES[gesture_id]
        ]

    if gesture_id == 3:
        return [
            ("open", keep_thumb_posture(BASE_GESTURES[gesture_id][0][1], thumb_values), BASE_GESTURES[gesture_id][0][2]),
            BASE_GESTURES[gesture_id][1],
        ]

    if gesture_id == 4:
        return [("open", keep_thumb_posture((0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0), thumb_values), 0.4)] + [
            (f"wave pass {pass_id} {name}", keep_thumb_posture(values, thumb_values), WAVE_HOLD_SECONDS)
            for pass_id in range(1, WAVE_REPEATS + 1)
            for name, values in WAVE_FINGER_STEPS
        ]

    if gesture_id in (5, 6, 7):
        return [
            (
                BASE_GESTURES[gesture_id][0][0],
                keep_thumb_posture(BASE_GESTURES[gesture_id][0][1], thumb_values),
                BASE_GESTURES[gesture_id][0][2],
            ),
        ] + [
            (
                BASE_GESTURES[gesture_id][i][0],
                BASE_GESTURES[gesture_id][i][1],
                BASE_GESTURES[gesture_id][i][2],
            )
            for i in range(1, len(BASE_GESTURES[gesture_id]))
        ]

    if gesture_id == 8:
        return BASE_GESTURES[gesture_id]

    return []


def get_return_step(thumb_values=None):
    return ("open together", keep_thumb_posture((0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0), thumb_values), 2.0)


def get_thumb_lead_return_step(thumb_values=None, hold_seconds=0.35):
    return (
        "thumb lead open",
        keep_thumb_posture((1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 0, 0, 0), thumb_values),
        hold_seconds,
    )


def get_ok_thumb_release_step(thumb_values=None):
    return (
        "ok thumb release",
        keep_thumb_posture((1000, 1000, 350, 350, 220, 220, 120, 120, 0, 0, 0), thumb_values),
        0.5,
    )


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


def ok_gesture_speed():
    return motor_speed((1000, 550, 500, 450, 350, 380, 380))


def ok_return_speed():
    return motor_speed((420, 520, 500, 450, 360, 420, 420))


def fist_close_speed():
    return motor_speed((1000, 900, 900, 900, 240, 240, 180))


def fist_return_speed():
    return motor_speed((1000, 1000, 1000, 1000, 900, 900, 900))


def thumb_touch_speed():
    return motor_speed((800, 800, 800, 800, 700, 750, 750))


def sequential_curl_speed():
    return motor_speed((650, 650, 650, 650, 400, 400, 400))


def cup_initial_speed():
    return motor_speed((650, 650, 650, 650, 420, 420, 420))


def middle_ring_pinch_speed():
    return motor_speed((480, 760, 640, 480, 650, 720, 720))


def temporary_speed_for_gesture(gesture_id):
    if gesture_id in (1, 2):
        return fist_close_speed()
    if gesture_id == 3:
        return ok_gesture_speed()
    if gesture_id == 6:
        return cup_initial_speed()
    if gesture_id == 7:
        return middle_ring_pinch_speed()
    if gesture_id == 8:
        return sequential_curl_speed()
    return thumb_touch_speed()


def connect_hand(hand_control, port, address):
    if address is None:
        # Search and connect to the first available Modbus slave
        connect_ret, slave_address = hand_control.connectModbusAuto(port)
        if not connect_ret:
            print("Please check if the dexterous hand is connected to the Modbus serial port")
            sys.exit(1)
        print("Auto connect Modbus success")
        return slave_address

    # Open the Modbus serial port
    if not hand_control.openModbusDevice(port, 4000000):
        print("Please check if the Modbus serial device is connected")
        sys.exit(1)
    print("Open Modbus serial port success")

    # Initialize the specified Modbus slave
    if hand_control.initModbusDevice(address) != 1:
        print("Please check if the dexterous hand is connected to the Modbus serial device")
        hand_control.disconnectModbus()
        sys.exit(1)
    print("Init Modbus slave success")
    return address


def prepare_position_mode(hand_control, slave_address):
    # Initialize motor position before enabling control
    if hand_control.initMotorPosition(slave_address) != 1:
        print("Init motor position failed")
        return False
    print("Init motor position success")

    # Check and disable motors before switching control mode
    if hand_control.isMotorEnabled(slave_address):
        if not hand_control.disableMotor(slave_address):
            print("Disable motor failed, cannot set position mode")
            return False
        print("Motor disabled before setting position mode")

    # Switch to position control mode
    if not hand_control.setMotorControlMode(slave_address, ControlMode.POSITION_CONTROL_MODE):
        print("Set position mode failed")
        return False
    print("Set position mode success")

    # Enable motors after the control mode is ready
    if not hand_control.enableMotor(slave_address):
        print("Enable motor failed")
        return False
    print("Enable motor success")

    # Check motor enable status before sending position commands
    if not hand_control.isMotorEnabled(slave_address):
        print("Motor status is disabled, unable to perform position control")
        return False
    print("Motor status is enabled")
    return True


def read_thumb_posture(hand_control, slave_address):
    # Read current joint angles and keep the thumb posture if needed
    result, joint_values = hand_control.getJointPositionsAngle(slave_address, False)
    if result != 1 or len(joint_values) != 11:
        return None
    return tuple(joint_values[8:11])


def execute_gesture(hand_control, slave_address, gesture_id):
    original_speed = None
    should_restore_speed = False

    if gesture_id in TEMPORARY_SPEED_GESTURES:
        # Save current speed configuration before using gesture-specific speed
        original_speed = hand_control.getMotorMaxVelocityConfig(slave_address)
        should_restore_speed = has_non_zero_speed(original_speed)
        # Set temporary max speed for the selected gesture
        if not hand_control.setMotorMaxVelocity(slave_address, temporary_speed_for_gesture(gesture_id)):
            print("Set temporary speed for gesture failed")
            return False
        print(f"Temporary speed set for gesture {gesture_id}")

    thumb_values = read_thumb_posture(hand_control, slave_address)
    if thumb_values is None:
        print("Read current thumb posture failed")
        return False
    print("Current thumb posture will be preserved when the selected gesture does not move the thumb")

    print(f"Execute gesture: {gesture_id}")
    for name, values, hold_seconds in build_gesture(gesture_id, thumb_values):
        # Send joint-angle target command for this gesture step
        if hand_control.setJointPositionsAngle(slave_address, values) != 1:
            print(f"Execute gesture step failed: {name}")
            return False
        print(f"Gesture step: {name}")
        time.sleep(hold_seconds)

    if gesture_id in (1, 2):
        # Set temporary return speed for fist-like gestures
        if not hand_control.setMotorMaxVelocity(slave_address, fist_return_speed()):
            print("Set temporary return speed for fist/stretch gesture failed")
            return False
        print("Temporary return speed set for fist/stretch gesture")

    if gesture_id == 3:
        name, values, hold_seconds = get_ok_thumb_release_step(thumb_values)
        # Release thumb first for the OK gesture return path
        if hand_control.setJointPositionsAngle(slave_address, values) != 1:
            print(f"Execute return step failed: {name}")
            return False
        print(f"Return step: {name}")
        time.sleep(hold_seconds)

        # Set temporary return speed for the OK gesture
        if not hand_control.setMotorMaxVelocity(slave_address, ok_return_speed()):
            print("Set temporary return speed for ok gesture failed")
            return False
        print("Temporary return speed set for ok gesture")

    if gesture_id in (1, 2):
        thumb_lead_hold_seconds = 0.3 if gesture_id == 1 else 0.35
        name, values, hold_seconds = get_thumb_lead_return_step(thumb_values, thumb_lead_hold_seconds)
        # Move the thumb away before opening the other fingers
        if hand_control.setJointPositionsAngle(slave_address, values) != 1:
            print(f"Execute return step failed: {name}")
            return False
        print(f"Return step: {name}")
        time.sleep(hold_seconds)

    print("Return to open posture together")
    name, values, hold_seconds = get_return_step(thumb_values)
    # Return all fingers to the open posture
    if hand_control.setJointPositionsAngle(slave_address, values) != 1:
        print(f"Execute return step failed: {name}")
        return False
    print(f"Return step: {name}")
    time.sleep(hold_seconds)

    if gesture_id in TEMPORARY_SPEED_GESTURES and should_restore_speed:
        # Restore the original speed configuration
        if not hand_control.setMotorMaxVelocity(slave_address, original_speed):
            print("Restore motor speed config failed")
            return False
        print("Motor speed config restored")

    return True


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--gesture",
        type=int,
        default=None,
        help="Gesture id. Currently implemented: 1-8 (1:fist,2:4-finger close,3:ok,4:wave,5:peace sign,6:cup initial grasp,7:middle-ring pinch,8:sequential curl)",
    )
    parser.add_argument(
        "--list-gestures",
        "--list",
        action="store_true",
        help="Print all implemented gestures and exit.",
    )
    parser.add_argument(
        "--all-gestures",
        action="store_true",
        help="Connect to the hand and run gestures 1-8 in order.",
    )
    parser.add_argument("--port", default="/dev/ttyUSB0", help="Modbus serial port, default: /dev/ttyUSB0")
    parser.add_argument(
        "--addr",
        type=lambda value: int(value, 0),
        default=None,
        help="Optional Modbus slave address. If omitted, connectModbusAuto is used.",
    )
    args = parser.parse_args()

    if args.list_gestures:
        print_gesture_help()
        sys.exit(0)

    if args.all_gestures and args.gesture is not None:
        print("Use either --gesture or --all-gestures")
        sys.exit(1)

    if not args.all_gestures and args.gesture not in IMPLEMENTED_GESTURES:
        print("Please select an implemented gesture, for example: --gesture 1")
        print_gesture_help()
        sys.exit(1)

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

    gesture_success = True
    gesture_ids = ALL_GESTURE_IDS if args.all_gestures else (args.gesture,)
    for index, gesture_id in enumerate(gesture_ids):
        if not execute_gesture(hand_control, slave_address, gesture_id):
            gesture_success = False
            break

    # Disable motors before disconnecting
    hand_control.disableMotor(slave_address)
    print("Disable motor success")

    # Close the Modbus serial port
    is_success = hand_control.disconnectModbus()
    print(f"Disconnect: {'success' if is_success else 'failed'}")
    sys.exit(0 if gesture_success and is_success else 1)
