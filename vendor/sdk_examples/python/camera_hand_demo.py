"""Camera and hand status integrated demo.

Steps:
    1. Connect to a Modbus slave automatically or by address.
    2. Query hand type and online status.
    3. Connect camera and capture frames for 10 seconds through SDK getFrame.
    4. Display frames in an OpenCV window.
    5. Release camera and close the Modbus serial port.
"""

import argparse
import sys
import time

import cv2
from pxdex.dh15 import DexH15Camera, DexH15Control


STREAM_SECONDS = 10
FRAME_INTERVAL_SECONDS = 1.0 / 30.0


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


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Capture camera frames for 10 seconds through SDK getFrame.")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="Modbus serial port, default: /dev/ttyUSB0")
    parser.add_argument(
        "--camera",
        default="/dev/video4",
        help="Camera device, default: /dev/video4. Use v4l2-ctl --list-devices to find the actual node.",
    )
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

    # Query hand type and online status
    hand_type = hand_control.getHandType(slave_address)
    print(f"Modbus device address: 0x{slave_address:02x}")
    print(f"Hand type: {str(hand_type).split('.')[-1]}")
    print(f"Online: {'yes' if hand_control.isModbusDeviceConnected(slave_address) else 'no'}")

    # Create camera control instance
    camera_control = DexH15Camera()
    # Open the camera device
    camera_control.connectCameraDevice(args.camera)
    # Set the resolution and frame rate
    if not camera_control.setCameraConfig(640, 480, 30):
        print("Set camera config failed, continue capturing with current config")

    print(f"Start camera stream for {STREAM_SECONDS} seconds")
    frame_count = 0
    window_created = False
    end_time = time.monotonic() + STREAM_SECONDS
    try:
        while time.monotonic() < end_time:
            # Capture one frame through the SDK camera interface
            frame = camera_control.getFrame()
            if getattr(frame, "size", 0) == 0:
                print("Get empty frame")
                break
            if frame_count == 0:
                print(f"Frame shape: {frame.shape}")
            frame_count += 1
            cv2.imshow("camera_hand_demo", frame)
            window_created = True
            if cv2.waitKey(1) == 27:
                break
            time.sleep(FRAME_INTERVAL_SECONDS)
        print(f"Captured frames: {frame_count}")
    finally:
        if window_created:
            cv2.destroyWindow("camera_hand_demo")
        # Release the camera device
        camera_control.releaseCameraDevice()
        print("Release camera device success")

    # Close the Modbus serial port
    result = hand_control.disconnectModbus()
    print(f"Disconnect: {'success' if result else 'failed'}")
