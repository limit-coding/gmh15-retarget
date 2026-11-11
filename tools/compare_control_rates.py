"""Run the backed-up and optimized controllers sequentially with identical input."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import urllib.request

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'runs/performance-20261107'
BASE = ROOT / 'backups/before-frequency-20261107'


def run(name, hz, seconds, source):
    env = dict(os.environ, GMH15_USB_TRACE=str(OUT / (name+'-usb.txt')))
    with (OUT / (name+'-server.log')).open('w') as log:
        cmd = [sys.executable, str(source / 'scripts/run_mac_hand.py'), '--motor-only', '--no-render', '--allow-motion']
        if hz != 10:
            cmd += ['--control-hz', str(hz)]
        proc = subprocess.Popen(cmd, cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT)
        try:
            for _ in range(60):
                if proc.poll() is not None:
                    raise RuntimeError('Server exited: ' + name)
                try:
                    with urllib.request.urlopen('http://127.0.0.1:8765/api/state', timeout=1) as response:
                        state = json.load(response)
                    if state['hands']['right'].get('connected'):
                        break
                except (OSError, ValueError, KeyError):
                    pass
                time.sleep(.5)
            else:
                raise RuntimeError('Server unavailable')
            cmd = [sys.executable, str(ROOT / 'tools/benchmark_teleop.py'), '--hz', str(hz),
                   '--seconds', str(seconds), '--out', str(OUT / (name+'.jsonl'))]
            if hz == 10:
                cmd += ['--legacy-driver', str(BASE / 'bare_hand_teleop/drivers.py')]
            with (OUT / (name+'-benchmark.log')).open('w') as bench_log:
                subprocess.run(cmd, cwd=ROOT, stdout=bench_log, stderr=subprocess.STDOUT, check=True)
            print(name, (OUT / (name+'.jsonl.summary.json')).read_text(), flush=True)
        finally:
            if proc.poll() is None:
                proc.send_signal(signal.SIGINT)
                try:
                    proc.wait(timeout=20)
                except subprocess.TimeoutExpired:
                    subprocess.run(['docker', 'stop', '-t', '8', 'gmh15-mac'], check=False)
                    proc.wait(timeout=10)


run('comparison-original', 10, 20, BASE)
run('comparison-optimized', 30, 40, ROOT)
