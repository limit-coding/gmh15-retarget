#!/usr/bin/env python3
"""Local, single-runner web controls for the verified Mac SDK examples."""
from collections import deque
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import argparse
import json
from pathlib import Path
import secrets
import signal
import subprocess
import sys
import threading
import time
from urllib.parse import urlsplit
import webbrowser

ROOT = Path(__file__).resolve().parents[1]
EXAMPLES = [
    dict(id="gesture1", title='握拳与张开', kind="官方 gesture_demo.py · 1", duration='约 8 秒',
         description='四指与拇指合拢握拳，再按官方顺序张开。', args=["--gesture", "1"]),
    dict(id="gesture2", title='逐指弯曲', kind="官方 gesture_demo.py · 2", duration='约 14 秒',
         description='食指、中指、无名指、小指依次弯曲，再收拢拇指。', args=["--gesture", "2"]),
    dict(id="gesture3", title='OK 手势', kind="官方 gesture_demo.py · 3", duration='约 9 秒',
         description='拇指与食指靠近，完成 OK 手势后张开。', args=["--gesture", "3"]),
    dict(id="gesture4", title='波浪', kind="官方 gesture_demo.py · 4", duration='约 8 秒',
         description='食指到小指依次运动，循环 5 遍后张开。', args=["--gesture", "4"]),
    dict(id="gesture5", title='剪刀手', kind="官方 gesture_demo.py · 5", duration='约 8 秒',
         description='食指、中指保持伸展，其余手指弯曲。', args=["--gesture", "5"]),
    dict(id="gesture6", title='杯形初始抓握', kind="官方 gesture_demo.py · 6", duration='约 8 秒',
         description='运行官方杯形抓握姿态，随后张开。测试时手里不放物体。', args=["--gesture", "6"]),
    dict(id="gesture7", title='中指与无名指捏合', kind="官方 gesture_demo.py · 7", duration='约 8 秒',
         description='运行官方预备姿态与中指、无名指捏合动作。', args=["--gesture", "7"]),
    dict(id="gesture8", title='分级卷指', kind="官方 gesture_demo.py · 8", duration='约 8 秒',
         description='中指、无名指、小指分级弯曲，拇指竖起，随后张开。', args=["--gesture", "8"]),
    dict(id='position', title='官方位置往返', kind='官方例程 · Mac 适配', duration='约 8 秒',
         description='初始化后，小幅弯曲并回零。先用它听听官方控制时是否也有卡卡声。',
         args=['--official-only']),
    dict(id='continuous10', title='连续开合 · 10 Hz', kind='官方 SDK · 对照测试', duration='约 16 秒',
         description='先做位置往返，再以 10 Hz 连续小幅开合 8 秒，用来对比声音和连贯程度。',
         args=['--hz', '10']),
    dict(id='continuous30', title='连续开合 · 30 Hz', kind='官方 SDK · 对照测试', duration='约 16 秒',
         description='先做位置往返，再以 30 Hz 连续小幅开合 8 秒，与 10 Hz 使用同一条轨迹。',
         args=['--hz', '30']),
    dict(id='usb', title='官方 USB 连接检查', kind='官方例程 · Mac 适配', duration='约 3 秒',
         description='连接设备、执行位置初始化并检查在线。初始化也可能使手指运动。',
         args=['--usb-example']),
]


class Runner:
    def __init__(self, command=None, check_device=True, log_dir=None):
        self.command = command or [sys.executable, '-u', str(ROOT/'tools/run_sdk_performance.py')]
        self.check_device = check_device
        self.lock = threading.Lock()
        self.logs = deque(maxlen=240)
        self.phase = 'idle'
        self.title = ''
        self.example = None
        self.proc = None
        self.worker = None
        self.stop_requested = False
        self.disabled = False
        self.message = '选择一个例程开始。'
        self.run = 0
        self.log_path = None
        self.log_dir = Path(log_dir) if log_dir else ROOT/'runs/example_web'
        self.last_progress = 0

    def log(self, line):
        line = line.rstrip()[:2000]
        with self.lock:
            if self.log_path:
                with self.log_path.open('a') as file:
                    file.write(line+'\n')
            try:
                event = json.loads(line)
                kind = event.get('event')
                if kind == 'disabled' and event.get('confirmed') is True:
                    self.disabled = True
                    line = '电机失能已确认。'
                elif kind == 'connected':
                    line = '已连接右手：'+event.get('sn','')
                elif kind == 'gesture_start':
                    line = '开始官方手势：'+self.title
                elif kind == 'gesture_completed':
                    line = '官方手势已完成，正在失能。'
                elif kind == 'gesture_target':
                    if event.get('limited'):
                        line = '拇指幅度按本机实测行程限制。'
                    else:
                        return
                elif kind == 'official_position_demo_start':
                    line = '初始化完成，开始官方位置往返。'
                elif kind == 'official_target':
                    line = '小幅弯曲。' if event.get('phase') == 'official_out' else '返回张开位置。'
                elif kind == 'official_usb_example_completed':
                    line = '官方 USB 连接检查完成。'
                elif kind == 'target':
                    now = time.monotonic()
                    if now-self.last_progress < 1:
                        return
                    self.last_progress = now
                    line = f'连续开合中（{event.get("hz", "")} Hz）。'
            except (ValueError, AttributeError):
                pass
            self.logs.append(line)

    def snapshot(self):
        with self.lock:
            busy = self.phase in ('starting', 'running', 'stopping')
            return dict(phase=self.phase, busy=busy, title=self.title, example=self.example,
                        message=self.message, disabled_confirmed=self.disabled,
                        logs=list(self.logs), run=self.run,
                        log_file=str(self.log_path) if self.log_path else None)

    def start(self, example_id):
        example = next((x for x in EXAMPLES if x['id'] == example_id), None)
        if example is None:
            raise ValueError('未知例程')
        with self.lock:
            if self.phase in ('starting', 'running', 'stopping'):
                raise ValueError('当前例程还未结束，请先停止并等待清理完成。')
            self.phase, self.title, self.example = 'starting', example['title'], example_id
            self.stop_requested, self.disabled = False, False
            self.message = '正在连接灵巧手…'
            self.logs.clear()
            self.last_progress = 0
            self.run += 1
            stamp = datetime.now().strftime('%Y%m%d-%H%M%S')+'-'+secrets.token_hex(3)
            folder = self.log_dir
            folder.mkdir(parents=True, exist_ok=True)
            self.log_path = folder/(stamp+'.log')
            self.worker = threading.Thread(target=self._run, args=(example,stamp), daemon=True)
            self.worker.start()

    def _run(self, example, stamp):
        try:
            if self.check_device:
                result = subprocess.run(['docker','ps','--format','{{.Names}}'], capture_output=True,
                                        text=True, timeout=5, check=True)
                if any(n in result.stdout.splitlines() for n in ('gmh15-mac','gmh15-sdk-comparison')):
                    raise RuntimeError('灵巧手正被其他程序使用，请先关闭遥控程序和旧版灵巧手服务。')
            with self.lock:
                if self.stop_requested:
                    self.phase, self.message = 'stopped', '已取消启动。'
                    return
                self.proc = subprocess.Popen(self.command+['--allow-motion', *example['args'], '--out', 'web-'+stamp],
                    cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                    text=True, errors='replace', bufsize=1, start_new_session=True)
                proc = self.proc
                self.phase, self.message = 'running', '例程运行中；可以随时点击停止。'
            timer = threading.Timer(45, self.stop)
            timer.start()
            try:
                for line in proc.stdout:
                    self.log(line)
                code = proc.wait()
            finally:
                timer.cancel()
                proc.stdout.close()
            with self.lock:
                if self.disabled and (code == 0 or self.stop_requested):
                    self.phase = 'stopped' if self.stop_requested else 'completed'
                    self.message = '已停止，电机失能已确认。可以选择下一个例程。' if self.stop_requested else '运行完成，电机失能已确认。可以选择下一个例程。'
                else:
                    self.phase = 'error'
                    self.message = ('运行异常；电机失能已确认。请查看日志。' if self.disabled else
                                    '例程未完成或失能未确认，请查看日志并检查设备。')
                self.proc = None
        except Exception as exc:
            self.log(str(exc))
            with self.lock:
                self.phase, self.message = 'error', str(exc)
                self.proc = None

    def stop(self):
        with self.lock:
            if self.phase not in ('starting', 'running', 'stopping'):
                return
            self.stop_requested = True
            self.phase, self.message = 'stopping', '正在停止并等待电机失能确认…'
            if self.proc and self.proc.poll() is None:
                try:
                    self.proc.send_signal(signal.SIGTERM)
                except ProcessLookupError:
                    pass

    def close(self):
        self.stop()
        if self.worker:
            self.worker.join(timeout=25)


def make_server(port=8767, runner=None):
    runner = runner or Runner()
    token = secrets.token_urlsafe(32)
    instance = secrets.token_hex(8)
    page = (ROOT/'mac_driver/examples.html').read_text().replace('__TOKEN__',token).replace('__INSTANCE__',instance)
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass
        def reply(self, value, status=200, html=False):
            data = value.encode() if html else json.dumps(value,ensure_ascii=False).encode()
            self.send_response(status)
            self.send_header('Content-Type','text/html; charset=utf-8' if html else 'application/json; charset=utf-8')
            self.send_header('Content-Length',str(len(data)))
            self.send_header('Cache-Control','no-store')
            self.send_header('X-Content-Type-Options','nosniff')
            self.end_headers()
            try:
                self.wfile.write(data)
            except (BrokenPipeError,ConnectionResetError):
                pass
        def local(self):
            try:
                return urlsplit('http://'+self.headers.get('Host','')).hostname in ('localhost','127.0.0.1','::1')
            except ValueError:
                return False
        def do_GET(self):
            if not self.local():
                return self.reply({'error':'仅允许本机访问'},403)
            path=urlsplit(self.path).path
            if path == '/':
                return self.reply(page,html=True)
            if path == '/api/state':
                return self.reply(dict(**runner.snapshot(), instance=instance, examples=[{k:v for k,v in x.items() if k != 'args'} for x in EXAMPLES]))
            return self.reply({'error':'Not found'},404)
        def do_POST(self):
            try:
                origin=self.headers.get('Origin')
                # Embedded browsers may send Origin: null, and localhost/127.0.0.1
                # can differ across the browser's local proxy. The per-page token
                # remains required; explicit non-local web origins stay blocked.
                origin_host = urlsplit(origin).hostname if origin not in (None, '', 'null') else None
                if not self.local() or (origin not in (None, '', 'null') and
                        (origin_host not in ('localhost', '127.0.0.1', '::1') or
                         urlsplit(origin).scheme not in ('http', 'https'))):
                    raise PermissionError('请求来源无效')
                length=int(self.headers.get('Content-Length','0'))
                if not 0 < length <= 2048:
                    raise ValueError('请求长度无效')
                data=json.loads(self.rfile.read(length))
                if not isinstance(data,dict) or not secrets.compare_digest(str(data.get('token','')),token):
                    raise PermissionError('请刷新页面后再操作')
                if self.path == '/api/start':
                    runner.start(data.get('example'))
                elif self.path == '/api/stop':
                    runner.stop()
                else:
                    return self.reply({'error':'Not found'},404)
                self.reply({'ok':True})
            except PermissionError as exc:
                self.reply({'error':str(exc)},403)
            except (ValueError,TypeError) as exc:
                self.reply({'error':str(exc)},409)
    server=ThreadingHTTPServer(('127.0.0.1',port),Handler)
    server.runner=runner
    return server


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port',type=int,default=8767)
    ap.add_argument('--open',action='store_true')
    args=ap.parse_args()
    try:
        server=make_server(args.port)
    except OSError:
        if args.open:
            webbrowser.open(f'http://127.0.0.1:{args.port}/')
        raise SystemExit('端口已被占用；若例程页面已启动，可直接使用。')
    url=f'http://127.0.0.1:{server.server_port}/'
    print('例程控制台：'+url,flush=True)
    print('打开网页不会启动电机。Ctrl+C 关闭控制台并停止当前例程。',flush=True)
    def stop(*_):
        raise KeyboardInterrupt()
    signal.signal(signal.SIGTERM,stop)
    signal.signal(signal.SIGINT,stop)
    if args.open:
        webbrowser.open(url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.runner.close()
        server.server_close()


if __name__ == '__main__':
    main()
