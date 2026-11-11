#!/usr/bin/env python3
"""Run the original ARM64 SDK locally on Mac with a native USB transport."""
import argparse
import os
from pathlib import Path
import secrets
import signal
import subprocess
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'mac_driver'))
from serial_bridge import SerialBridge, discover


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--side', choices=('left', 'right'), default='right')
    parser.add_argument('--device', help='Mac /dev/cu.usbmodem…; auto-detects one STM32 device')
    parser.add_argument('--probe', action='store_true', help='Read identity/state once, then exit')
    parser.add_argument('--allow-motion', action='store_true', help='Unlock explicit UI initialization/arming; never auto-arm')
    parser.add_argument('--no-render', action='store_true')
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--bridge-port', type=int, default=9872)
    parser.add_argument('--camera', type=int, help='AVFoundation camera index (ffmpeg -f avfoundation -list_devices true -i "")')
    parser.add_argument('--init-diag', action='store_true',
                        help='One-shot: clear faults, initMotorPosition once, then disable. May move fingers.')
    parser.add_argument('--motor-only', action='store_true', help='Skip tactile SDK queries for isolated motor diagnostics')
    parser.add_argument('--control-hz', type=float, default=10, help='Motor control frequency, 5-30 Hz; default 10')
    args = parser.parse_args()
    if args.motor_only and args.camera is not None:
        parser.error('Motor-only mode does not open a camera; omit --camera.')
    if not 5 <= args.control_hz <= 30 or (args.control_hz != 10 and not args.motor_only):
        parser.error('5-30 Hz supported; non-default rates require --motor-only')
    devices = discover()
    device = args.device
    if not device:
        if len(devices) != 1:
            parser.error(f'Expected exactly one STM32 controller, found {devices}. Use --device for multiple hands.')
        device = devices[0]
    # A double-click launcher may have just started OrbStack.
    for attempt in range(30):
        try:
            ready = subprocess.run(['docker', 'info'], stdout=subprocess.DEVNULL,
                                   stderr=subprocess.DEVNULL, timeout=3).returncode == 0
        except subprocess.TimeoutExpired:
            ready = False
        if ready:
            break
        if attempt == 0:
            print('Waiting for the local Linux environment…', flush=True)
        time.sleep(1)
    else:
        parser.error('OrbStack is not ready. Open OrbStack and try again.')
    subprocess.run(['docker', 'image', 'inspect', 'gmh15-teleop:3.9.0'], check=True, stdout=subprocess.DEVNULL)
    token = secrets.token_hex(32)
    bridge = SerialBridge(device, token, args.bridge_port)
    thread = threading.Thread(target=bridge.serve, daemon=True)
    thread.start()
    camera = None
    print(f'Mac USB: {device}; mode: ' + ('read-only probe' if args.probe else 'disarmed, UI control available' if args.allow_motion else 'motion locked'), flush=True)
    cmd = ['docker', 'run', '--rm', '--init', '--name', 'gmh15-mac', '--stop-timeout', '8',
           '--cap-drop', 'ALL', '--security-opt', 'no-new-privileges',
           '-e', 'GMH15_BRIDGE_TOKEN', '-e', f'GMH15_BRIDGE_PORT={args.bridge_port}',
           '-e', f'GMH15_CONTROL_HZ={args.control_hz}',
           # optional speed overrides (see mac_driver/teleop_control.py)
           *[x for k in ('GMH15_SLEW_DEG_S', 'GMH15_MOTOR_SPEED', 'GMH15_MAX_TRACKING_ERROR_DEG',
                         'GMH15_MOTOR_SLEW_CPS', 'GMH15_MOTOR_MAX_ERROR', 'GMH15_CURRENT_STOP',
                         'GMH15_GRIP_CURRENT', 'GMH15_CONTACT_LEAD', 'GMH15_HOLD_MAX_S')
             if k in os.environ for x in ('-e', k)],
           '-v', f'{ROOT / "mac_driver"}:/opt/teleop:ro',
           '-v', f'{ROOT / "vendor/yubi-pi_gmh15-sdk/models"}:/opt/models:ro']
    if not (args.probe or args.init_diag):
        cmd += ['-p', f'127.0.0.1:{args.port}:8765']
    cmd += ['--entrypoint', 'python3', 'gmh15-teleop:3.9.0', '-u', '/opt/teleop/container_entry.py',
            'python3', '-u']
    if args.probe:
        cmd += ['/opt/teleop/probe.py']
    elif args.init_diag:
        cmd += ['/opt/teleop/init_diag.py', {'right': 'H15R49G00039', 'left': 'H15L49G00062'}[args.side]]
    else:
        cmd += ['/opt/teleop/teleop_server.py', '--bind', '0.0.0.0', '--hands', args.side,
                f'--{args.side}-port', '/tmp/gmh15-serial']
        if args.camera is None:
            cmd += ['--no-camera']
        else:
            from camera_relay import CameraRelay
            camera = CameraRelay(args.camera, token)
            camera.start()
            cmd += ['--camera-url', 'http://host.docker.internal:9873' + camera.path]
        if args.allow_motion:
            cmd += ['--allow-motion']
        if args.no_render:
            cmd += ['--no-render']
        if args.motor_only:
            cmd += ['--motor-only']
    env = dict(os.environ, GMH15_BRIDGE_TOKEN=token)
    child = None
    stopping = threading.Event()
    def stop(*_):
        stopping.set()
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    try:
        child = subprocess.Popen(cmd, env=env)
        while child.poll() is None and not stopping.is_set():
            if bridge.closed.wait(.2):
                # Give a successful one-shot query time to exit normally.
                try:
                    child.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    stopping.set()
                break
        if child.poll() is None:
            subprocess.run(['docker', 'stop', '-t', '8', 'gmh15-mac'], stdout=subprocess.DEVNULL, timeout=15)
        return child.wait(timeout=5)
    finally:
        bridge.close()
        thread.join(timeout=2)
        if bridge.error:
            print(f'USB transport: {bridge.error}', file=sys.stderr)
        if camera:
            camera.close()
        print(f'USB bytes: to hand={bridge.tx}, from hand={bridge.rx}', flush=True)


if __name__ == '__main__':
    sys.exit(main())
