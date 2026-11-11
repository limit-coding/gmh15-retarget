"""Run an isolated SDK example on Mac; never shares a port with teleop."""
from pathlib import Path
import os
import secrets
import signal
import subprocess
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'mac_driver'))
from serial_bridge import SerialBridge, discover

CONTAINER = 'gmh15-sdk-comparison'
# Signal only the SDK child: its parent must keep forwarding USB during cleanup.
SIGNAL_SDK = """
import os, signal, sys
sent = False
for name in os.listdir('/proc'):
    if name.isdigit():
        try:
            args = open('/proc/'+name+'/cmdline','rb').read().split(b'\\0')
            if len(args) > 2 and args[2] == b'/opt/teleop/sdk_performance.py':
                os.kill(int(name), signal.SIGTERM)
                sent = True
        except (FileNotFoundError, ProcessLookupError):
            pass
sys.exit(0 if sent else 1)
"""


def main():
    devices = discover()
    if len(devices) != 1:
        raise SystemExit(f'Expected one STM32 device: {devices}')
    names = subprocess.check_output(['docker', 'ps', '--format', '{{.Names}}'], timeout=5).decode().splitlines()
    if 'gmh15-mac' in names or CONTAINER in names:
        raise SystemExit('灵巧手正被其他程序使用，请先关闭遥控或其他例程。')
    token = secrets.token_hex(32)
    bridge = SerialBridge(devices[0], token)
    thread = threading.Thread(target=bridge.serve, daemon=True)
    thread.start()
    stopped = threading.Event()
    def stop(*_):
        stopped.set()
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    output = ROOT / 'runs/performance-20261107'
    output.mkdir(parents=True, exist_ok=True)
    child = None
    try:
        child = subprocess.Popen(['docker', 'run', '--rm', '--init', '--name', CONTAINER,
            '-e', 'GMH15_BRIDGE_TOKEN', '-v', f'{ROOT / "mac_driver"}:/opt/teleop:ro',
            '-v', f'{ROOT / "vendor/sdk_examples/python"}:/opt/examples:ro',
            '-v', f'{output}:/out', '--entrypoint', 'python3', 'gmh15-teleop:3.9.0',
            '-u', '/opt/teleop/container_entry.py', 'python3', '-u', '/opt/teleop/sdk_performance.py',
            *sys.argv[1:]], env=dict(os.environ, GMH15_BRIDGE_TOKEN=token))
        stop_at = None
        last_signal = 0
        while child.poll() is None:
            now = time.monotonic()
            if stopped.is_set():
                if stop_at is None:
                    stop_at = now
                    print('正在停止例程并确认电机失能…', flush=True)
                if now-last_signal > 1:
                    result = subprocess.run(['docker', 'exec', CONTAINER, 'python3', '-c', SIGNAL_SDK],
                                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=4)
                    last_signal = float('inf') if result.returncode == 0 else now
                if now-stop_at > 12:
                    print('失能清理超时，请现场检查灵巧手。', flush=True)
                    subprocess.run(['docker', 'stop', '-t', '3', CONTAINER], timeout=8)
                    break
            stopped.wait(.1) if not stopped.is_set() else time.sleep(.1)
        return child.wait(timeout=10)
    finally:
        if child is not None and child.poll() is None:
            subprocess.run(['docker', 'stop', '-t', '3', CONTAINER], timeout=8)
            child.wait(timeout=10)
        bridge.close()
        thread.join(timeout=2)


if __name__ == '__main__':
    sys.exit(main())
