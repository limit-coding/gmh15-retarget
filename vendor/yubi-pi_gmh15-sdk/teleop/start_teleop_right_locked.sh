#!/bin/sh
# User requested right hand only and NO motor commands.
set -eu
base=/home/limit/gmh15-sdk
stage="$base/teleop"
right=$(readlink -f /dev/serial/by-path/platform-xhci-hcd.0-usb-0:1.3:1.0)
rightcam=$(readlink -f /dev/v4l/by-id/usb-Image+_Usb_HD_DH-0000007-video-index0)
test -c "$right" && test -c "$rightcam"
if sudo -n docker ps --format '{{.Names}}' | grep -Eq '^gmh15-(tactile|teleop)$'; then
    echo 'Existing hand container is running; refusing serial contention.' >&2
    exit 1
fi
exec sudo -n docker run -d --name gmh15-teleop --restart no --cap-drop ALL --security-opt no-new-privileges \
    --device "$right:/dev/ttyACM1" --device "$rightcam:/dev/video2" \
    -p 127.0.0.1:8765:8765 -v "$stage:/opt/teleop:ro" -v "$base/models:/opt/models:ro" \
    --entrypoint python3 gmh15-teleop:3.9.0 -u /opt/teleop/teleop_server.py --bind 0.0.0.0 --port 8765 --hands right
