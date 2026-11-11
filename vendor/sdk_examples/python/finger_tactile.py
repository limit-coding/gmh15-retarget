"""Tactile resultant force and temperature example.

Steps:
    1. Connect to a Modbus slave automatically or by address.
    2. Query the hand type.
    3. Calibrate tactile sensors.
    4. Read resultant force, normal force, and temperature.
    5. Close the Modbus serial port.
"""

import argparse
import sys
import time

from pxdex.dh15 import DexH15Control


FINGER_NAMES = ["index", "middle", "ring", "pinky", "thumb"]
READ_HZ = 10
READ_DURATION_SECONDS = 15


def finger_name(index):
    if index < len(FINGER_NAMES):
        return FINGER_NAMES[index]
    return f"finger_{index}"


def print_resultant_force(force_points):
    if not force_points:
        print("Resultant force: no data")
        return

    total_normal_force = 0
    print("Resultant force, unit: 0.1N")
    for index, force_point in enumerate(force_points):
        total_normal_force += force_point.z
        print(
            f"  {finger_name(index):>6} "
            f"x:{force_point.x:>5} y:{force_point.y:>5} normal(z):{force_point.z:>5}"
        )
    print(f"  total normal(z): {total_normal_force}")


def print_temperature(temperatures):
    if not temperatures:
        print("Temperature: no data")
        return

    print("Temperature")
    for index, temperature in enumerate(temperatures):
        name = temperature.finger_name or finger_name(index)
        print(f"  {name:>6} sensor:{temperature.sensor_ID} temp:{temperature.temperature}")


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

    # 4. Calibrate tactile sensors before reading force data
    if not hand_control.calibrateSensor(slave_address):
        print("Sensor calibration failed, continue reading raw tactile data")
    else:
        print("Sensor calibration success")

    print(f"\nStart reading tactile data (frequency: {READ_HZ}Hz, duration: {READ_DURATION_SECONDS}s)")
    print("Tip: Press the fingertip sensor to see value changes")

    sample_index = 0
    end_time = time.monotonic() + READ_DURATION_SECONDS
    try:
        while time.monotonic() < end_time:
            sample_index += 1
            print(f"\nSample {sample_index}")
            # 5. Read resultant force from each finger
            try:
                force_points = hand_control.getFingerResultantForce(slave_address)
            except RuntimeError as exc:
                print(f"Read resultant force failed: {exc}")
                force_points = []

            # 6. Read tactile sensor temperature from each finger
            try:
                temperatures = hand_control.getTemperature(slave_address)
            except RuntimeError as exc:
                print(f"Read temperature failed: {exc}")
                temperatures = []

            print_resultant_force(force_points)
            print_temperature(temperatures)
            time.sleep(1.0 / READ_HZ)
    except KeyboardInterrupt:
        print("\nUser interrupted")
    finally:
        # 6. Close Modbus serial port
        ret = hand_control.disconnectModbus()
        print(f"Disconnect: {'success' if ret else 'failed'}")
