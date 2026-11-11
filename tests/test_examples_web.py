"""Lifecycle tests use a fake child process, never Docker or a robot."""
import importlib.util
import json
from pathlib import Path
import re
import sys
import threading
import time
import urllib.error
import urllib.request

import pytest

ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('examples_web',ROOT/'scripts/run_examples_web.py')
module=importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def wait_for(runner, predicate):
    deadline=time.monotonic()+3
    while time.monotonic()<deadline:
        state=runner.snapshot()
        if predicate(state):
            return state
        time.sleep(.01)
    raise AssertionError(state)


def fake_runner(tmp_path):
    script=tmp_path/'fake.py'
    script.write_text('''import signal,time,json,sys
stopped=False
def stop(*args):
    global stopped
    stopped=True
signal.signal(signal.SIGTERM,stop)
print('started',flush=True)
while not stopped: time.sleep(.01)
time.sleep(.15)
print(json.dumps(dict(event='disabled',confirmed=True)),flush=True)
sys.exit(130)
''')
    return module.Runner([sys.executable,'-u',str(script)],check_device=False,log_dir=tmp_path/'logs')


def test_stop_waits_for_disable_and_allows_next_run(tmp_path):
    runner=fake_runner(tmp_path)
    try:
        runner.start('position')
        wait_for(runner,lambda s:'started' in s['logs'])
        with pytest.raises(ValueError):
            runner.start('continuous30')
        runner.stop()
        assert runner.snapshot()['busy']
        with pytest.raises(ValueError):
            runner.start('continuous30')
        state=wait_for(runner,lambda s:not s['busy'])
        assert state['phase']=='stopped' and state['disabled_confirmed']
        runner.start('usb')
        wait_for(runner,lambda s:'started' in s['logs'])
        runner.stop()
        assert wait_for(runner,lambda s:not s['busy'])['disabled_confirmed']
    finally:
        runner.close()


def test_http_rejects_cross_site_and_unknown_examples(tmp_path):
    runner=fake_runner(tmp_path)
    server=module.make_server(0,runner)
    threading.Thread(target=server.serve_forever,daemon=True).start()
    base=f'http://127.0.0.1:{server.server_port}'
    try:
        page=urllib.request.urlopen(base).read().decode()
        token=re.search("const TOKEN='([^']+)'",page)[1]
        def post(path,body,origin=base):
            request=urllib.request.Request(base+path,data=json.dumps(body).encode(),headers={'Origin':origin,'Content-Type':'application/json'})
            try:
                return urllib.request.urlopen(request).status
            except urllib.error.HTTPError as exc:
                return exc.code
        assert post('/api/start',dict(token='bad',example='position'))==403
        assert post('/api/start',dict(token=token,example='position'),'http://evil.test')==403
        assert post('/api/stop',dict(token='bad'),'null')==403
        assert post('/api/stop',dict(token=token),'null')==200
        assert post('/api/stop',dict(token=token),'http://localhost:8767')==200
        assert post('/api/stop',dict(token=token),'http://127.0.0.1.evil.test')==403
        assert post('/api/start',dict(token=token,example='../../bad'))==409
        assert not runner.snapshot()['busy']
        assert post('/api/start',dict(token=token,example='position'))==200
        wait_for(runner,lambda s:'started' in s['logs'])
        assert post('/api/start',dict(token=token,example='usb'))==409
        assert post('/api/stop',dict(token=token))==200
        assert wait_for(runner,lambda s:not s['busy'])['disabled_confirmed']
    finally:
        runner.close()
        server.shutdown()
        server.server_close()


def test_stop_signals_sdk_child_without_stopping_usb_forwarder(monkeypatch):
    import ast
    import builtins
    import io
    import os
    tree=ast.parse((ROOT/'tools/run_sdk_performance.py').read_text())
    code=next(n.value.value for n in tree.body if isinstance(n,ast.Assign) and
              any(isinstance(t,ast.Name) and t.id=='SIGNAL_SDK' for t in n.targets))
    commands={
        '/proc/101/cmdline':b'python3\0-u\0/opt/teleop/container_entry.py\0python3\0-u\0/opt/teleop/sdk_performance.py\0',
        '/proc/102/cmdline':b'python3\0-u\0/opt/teleop/sdk_performance.py\0--allow-motion\0',
    }
    killed=[]
    monkeypatch.setattr(os,'listdir',lambda path:['101','102'])
    monkeypatch.setattr(builtins,'open',lambda path,mode:io.BytesIO(commands[path]))
    monkeypatch.setattr(os,'kill',lambda pid,sig:killed.append(pid))
    with pytest.raises(SystemExit) as exit_info:
        exec(code,{})
    assert exit_info.value.code == 0
    assert killed == [102]
