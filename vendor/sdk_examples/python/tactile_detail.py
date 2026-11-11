"""Complete tactile data example.

Steps:
    1. Connect to a Modbus slave automatically or by address.
    2. Query the hand type.
    3. Calibrate tactile sensors.
    4. Read all tactile interfaces.
    5. Print single-finger tactile data and press position force.
    6. Close the Modbus serial port.
"""

import argparse
import sys
import time

from pxdex.dh15 import DexH15Control, FingerType


READ_HZ = 2
READ_DURATION_SECONDS = 15
SUMMARY_LABEL_WIDTH = 22
FINGERS = [
    (FingerType.INDEX_FINGER, "index"),
    (FingerType.MIDDLE_FINGER, "middle"),
    (FingerType.RING_FINGER, "ring"),
    (FingerType.PINKY_FINGER, "pinky"),
    (FingerType.THUMB_FINGER, "thumb"),
]


def finger_name(index):
    if index < len(FINGERS):
        return FINGERS[index][1]
    return f"finger_{index}"


def summary_prefix(label):
    return f"  {label:<{SUMMARY_LABEL_WIDTH}} | "


def print_resultant_summary(points):
    if not points:
        print(f"{summary_prefix('resultant_force')}fingers=0   total_normal=0     max_finger=none   max_normal=0")
        return

    total_normal_force = sum(point.z for point in points)
    max_index, max_point = max(enumerate(points), key=lambda item: abs(item[1].z))
    print(
        f"{summary_prefix('resultant_force')}"
        f"fingers={len(points):<3} total_normal={total_normal_force:<5} "
        f"max_finger={finger_name(max_index):<6} max_normal={max_point.z}"
    )


def print_temperature_summary(temperatures):
    if not temperatures:
        print(f"{summary_prefix('temperature')}sensors=0   min=0.0   max=0.0   avg=0.0")
        return

    values = [temperature.temperature for temperature in temperatures]
    print(
        f"{summary_prefix('temperature')}"
        f"sensors={len(values):<3} min={min(values):<5.1f} "
        f"max={max(values):<5.1f} "
        f"avg={(sum(values) / len(values)):.1f}"
    )


def print_distributed_summary(title, points):
    if not points:
        print(f"{summary_prefix(title)}points=0   total_normal=0     max_index=none   max_normal=0")
        return

    total_normal_force = sum(point.z for point in points)
    max_index, max_point = max(enumerate(points), key=lambda item: abs(item[1].z))
    print(
        f"{summary_prefix(title)}"
        f"points={len(points):<3} total_normal={total_normal_force:<5} "
        f"max_index={max_index:<5} max_normal={max_point.z}"
    )


def is_active_press_point(point):
    return point.force_value != 0 or point.x != 0 or point.y != 0 or point.z != 0


def print_press_position_force(points):
    if not points:
        print(f"{summary_prefix('press_position_force')}returned=0   active=0   max_model=none         max_force=0")
        return

    active_points = [(index, point) for index, point in enumerate(points) if is_active_press_point(point)]
    max_model = "none"
    max_force = 0
    for _, point in active_points:
        if abs(point.force_value) >= abs(max_force):
            max_force = point.force_value
            max_model = point.model_name or "unknown"
    print(
        f"{summary_prefix('press_position_force')}"
        f"returned={len(points):<3} active={len(active_points):<3} "
        f"max_model={max_model:<12} max_force={max_force}"
    )


def connect_hand(hand_control, port, address):
    if address is None:
        connect_ret, slave_address = hand_control.connectModbusAuto(port)
        if not connect_ret:
            print("Please check if the dexterous hand is connected to the Modbus serial port")
            sys.exit(1)
        print("Auto connect Modbus success")
        return slave_address

    if not hand_control.openModbusDevice(port, 4000000):
        print("Please check if the Modbus serial device is connected")
        sys.exit(1)
    print("Open Modbus serial port success")

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
        help="Optional Modbus slave address. If omitted, connectModbusAuto is used.",
    )
    args = parser.parse_args()

    # 1. Create serial control instance
    hand_control = DexH15Control()
    # 2. Connect to the selected hand. If --addr is omitted, connect to the first slave found.
    slave_address = connect_hand(hand_control, args.port, args.addr)

    # 3. Query the hand type
    hand_type = hand_control.getHandType(slave_address)
    print(f"Modbus device address: 0x{slave_address:02x}")
    print(f"Hand type: {str(hand_type).split('.')[-1]}")

    # 4. Calibrate tactile sensors before reading detailed tactile data
    if not hand_control.calibrateSensor(slave_address):
        print("Sensor calibration failed, continue reading raw tactile data")
    else:
        print("Sensor calibration success")

    print(f"\nStart reading complete tactile data (frequency: {READ_HZ}Hz, duration: {READ_DURATION_SECONDS}s)")

    sample_index = 0
    end_time = time.monotonic() + READ_DURATION_SECONDS
    try:
        while time.monotonic() < end_time:
            sample_index += 1
            print(f"\nSample {sample_index}")

            # 5. Read all tactile interfaces
            try:
                resultant_force = hand_control.getFingerResultantForce(slave_address)
            except RuntimeError as exc:
                print(f"Read resultant force failed: {exc}")
                resultant_force = []

            try:
                distributed_force = hand_control.getFingerDistributedForce(slave_address)
            except RuntimeError as exc:
                print(f"Read distributed force failed: {exc}")
                distributed_force = []

            try:
                temperatures = hand_control.getTemperature(slave_address)
            except RuntimeError as exc:
                print(f"Read temperature failed: {exc}")
                temperatures = []

            try:
                all_resultant_force, all_distributed_force, all_temperatures = hand_control.getAllTactileData(
                    slave_address
                )
            except RuntimeError as exc:
                print(f"Read all tactile data failed: {exc}")
                all_resultant_force, all_distributed_force, all_temperatures = [], [], []

            try:
                press_points = hand_control.getPressPositionForce(slave_address)
            except RuntimeError as exc:
                print(f"Read press position force failed: {exc}")
                press_points = []

            print_resultant_summary(resultant_force)
            print_distributed_summary("distributed_force", distributed_force)
            print_temperature_summary(temperatures)

            single_finger_count = 0
            single_finger_detail_points = 0
            max_finger_name = "none"
            max_finger_normal = 0
            max_finger_abs_normal = 0
            for finger_type, name in FINGERS:
                try:
                    finger_force, finger_detail, finger_temperature = hand_control.getFingerTactileData(
                        slave_address, finger_type
                    )
                    single_finger_count += 1
                    single_finger_detail_points += len(finger_detail)
                    current_abs_normal = abs(finger_force.z)
                    if current_abs_normal > max_finger_abs_normal:
                        max_finger_abs_normal = current_abs_normal
                        max_finger_normal = finger_force.z
                        max_finger_name = name
                except RuntimeError as exc:
                    print(f"  {name:>6} read failed: {exc}")
            print(
                f"{summary_prefix('single_finger_data')}"
                f"fingers={single_finger_count:<3} "
                f"detail_points={single_finger_detail_points:<3} "
                f"max_finger={max_finger_name:<6} "
                f"max_normal={max_finger_normal}"
            )

            print(
                f"{summary_prefix('all_tactile_data')}"
                f"resultant={len(all_resultant_force):<3} "
                f"distributed={len(all_distributed_force):<3} "
                f"temperature={len(all_temperatures)}"
            )
            # 7. Print active press position force points
            print_press_position_force(press_points)

            time.sleep(1.0 / READ_HZ)
    except KeyboardInterrupt:
        print("\nUser interrupted")
    finally:
        # 8. Close Modbus serial port
        ret = hand_control.disconnectModbus()
        print(f"Disconnect: {'success' if ret else 'failed'}")
