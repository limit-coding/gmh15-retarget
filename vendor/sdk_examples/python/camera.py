"""H15 camera example.

Steps:
    1. Create the camera control instance.
    2. Connect the camera device.
    3. Set the resolution and frame rate.
    4. Capture and display one frame.
    5. Release the camera device.
"""

import argparse

import pxdex.dh15 as hand_packge

# Camera interface usage reference example
if __name__ == "__main__":
    """
    Steps:
        1. Create camera control instance [required]
        2. Connect camera [required]
        3. Set camera parameters
        4. Get a frame
        5. Release camera interface [required]
    """
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--camera",
        default="/dev/video4",
        help="Camera device, default: /dev/video4. Use v4l2-ctl --list-devices to find the actual node.",
    )
    args = parser.parse_args()

    # 1. Create camera control instance
    camera_control = hand_packge.DexH15Camera()

    # Camera port, adjust according to actual situation
    camera_port = args.camera
    # 2. Connect camera
    camera_control.connectCameraDevice(camera_port)
    print("connect camera device success")

    # 3. Set camera parameters
    if camera_control.setCameraConfig(640, 480, 30):
        print("set camera config success")
    else:
        print("set camera config failed")

    # 4. Get a frame
    frame = camera_control.getFrame()
    camera_control.showImage("image", frame)

    # 5. Release camera interface
    camera_control.releaseCameraDevice()
    print("release camera device success")
