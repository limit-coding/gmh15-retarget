"""Tiny read-only status page for run_sim.py: http://127.0.0.1:8766/

/            HTML page (polls the two endpoints below)
/frame.jpg   latest composite frame (camera | sim | panel)
/state.json  per-DOF sim angle, value sent to hardware, hardware measured; driver status
POST /action {"action": arm|disarm|open|fist, "token": ...}  queued for the app's main loop
Bound to 127.0.0.1 only; actions need the per-run token embedded in the page and a local Origin.
"""
import json
import secrets
import threading
from urllib.parse import urlsplit
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

PAGE = """<!doctype html><html lang="zh"><head><meta charset="utf-8"><title>GMH15 retarget</title>
<style>
body{font:14px -apple-system,sans-serif;margin:16px;background:#111;color:#ddd}
img{max-width:100%;border:1px solid #333;border-radius:6px}
#st{font-size:18px;font-weight:600;margin:10px 0}
table{border-collapse:collapse;margin-top:8px}td,th{padding:4px 10px;text-align:right}
th{color:#999;font-weight:500}td:first-child,th:first-child{text-align:left}
.bar{position:relative;width:260px;height:14px;background:#222;border-radius:3px}
.bar span{position:absolute;top:0;height:14px;border-radius:3px;opacity:.85}
.s{background:#4a8cff}.h{background:#ff9f2e;height:7px!important}.m{background:#3fd27f;top:7px!important;height:7px!important}
.btns{margin:10px 0}.btns button{font-size:17px;padding:10px 22px;margin-right:10px;border:0;border-radius:8px;cursor:pointer;color:#fff}
#arm{background:#1f9d55}#stop{background:#d93025;font-weight:700}.cal{background:#444}
button:disabled{opacity:.35;cursor:default}#msg{color:#ffcf5c;margin-left:6px}
.armed{color:#ff5050}.dis{color:#aaa}.lgd span{display:inline-block;width:10px;height:10px;margin:0 4px 0 12px}
</style></head><body>
<div id="st">连接中…</div>
<div class="btns"><button id="arm" onclick="act('arm')">▶ 使能真机</button><button id="stop" onclick="act('disarm')">■ 停止</button>
<button id="hold" onclick="act('hold')" style="background:#8a5cf6">✊ 保持握持</button><button id="rel" onclick="act('release')" style="background:#c47f00">✋ 松开</button>
<button class="cal" onclick="act('open')">标定：张开</button><button class="cal" onclick="act('fist')">标定：握拳</button><span id="msg"></span></div>
<img id="im" src="/frame.jpg">
<div class="lgd"><span class="s"></span>仿真（=真机可达范围）<span style="background:#ff9f2e"></span>发给真机<span style="background:#3fd27f"></span>真机实测</div>
<table><thead><tr><th>关节</th><th>仿真<span class=u></span></th><th>发送<span class=u></span></th><th>实测<span class=u></span></th><th>真机上限<span class=u></span></th><th></th></tr></thead><tbody id="tb"></tbody></table>
<script>
const TOKEN="__TOKEN__",RUN="__RUN__";
async function act(a){
 document.getElementById('msg').textContent={arm:'正在使能…',disarm:'正在停止…',hold:'正在切换到保持…',release:'正在松开…'}[a]||'';
 try{const r=await (await fetch('/action',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({action:a,token:TOKEN})})).json();
  document.getElementById('msg').textContent=r.ok?'已发送':'失败: '+r.error;}catch(e){document.getElementById('msg').textContent='程序未运行';}
}
document.addEventListener('keydown',e=>{if(e.key===' '||e.key==='Escape'){e.preventDefault();act('disarm')}});
let UNIT='°';const f=v=>v==null?'—':(UNIT==='°'?v.toFixed(1):Math.round(v));
async function tick(){
 try{const d=await (await fetch('/state.json',{cache:'no-store'})).json();
  if(d.run&&d.run!==RUN){location.reload();return;}   // program restarted: new token
  UNIT=d.unit||'°';document.querySelectorAll('.u').forEach(e=>e.textContent=UNIT==='°'?'°':'('+UNIT+')');
  const a=d.hw&&d.hw.armed;
  const H=d.hw&&(d.hw.holding||d.hw.releasing);
  document.getElementById('arm').disabled=!d.hw||a||H; document.getElementById('stop').disabled=!d.hw;
  document.getElementById('hold').disabled=!a; document.getElementById('rel').disabled=!(a||H);
  if(d.msg)document.getElementById('msg').textContent=d.msg;
  document.getElementById('st').innerHTML=`<span class="${a?'armed':'dis'}">${d.hw?(d.hw.releasing?'正在松开…':d.hw.holding?'✊ 保持握持中（不跟随手）'+(d.hw.hold_remaining_s!=null?' · 剩余 '+Math.round(d.hw.hold_remaining_s/60)+' 分钟':''):a?'真机已使能（跟随中）':(d.hw.fault?'真机故障: '+d.hw.fault:'真机未使能（点「使能真机」开始）')):'仅仿真'}</span> · 追踪 ${d.state} · ${d.fps.toFixed(0)} fps`;
  let h='';d.rows.forEach(r=>{const W=260,sx=r.sim/r.sim_max*W,hx=r.sent==null?0:r.sent/r.sim_max*W,mx=r.meas==null?0:r.meas/r.sim_max*W;
   h+=`<tr><td>${r.name}</td><td>${f(r.sim)}</td><td>${f(r.sent)}</td><td>${f(r.meas)}</td><td>${f(r.hw_max)}</td><td><div class="bar"><span class="s" style="width:${sx}px"></span><span class="h" style="width:${hx}px"></span><span class="m" style="width:${mx}px"></span></div></td></tr>`});
  document.getElementById('tb').innerHTML=h;
 }catch(e){document.getElementById('st').textContent='程序未运行';}
 document.getElementById('im').src='/frame.jpg?'+Date.now();
}
setInterval(tick,200);tick();
</script></body></html>"""


class LiveWeb:
    def __init__(self, port=8766):
        self.lock = threading.Lock()
        self.jpeg = b""
        self.state = {"rows": [], "state": "-", "fps": 0.0, "hw": None, "msg": ""}
        self.token = secrets.token_hex(16)
        self.state["run"] = secrets.token_hex(4)   # public run id; page reloads when it changes
        self.actions = []
        self.port = port
        web = self

        class H(BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def do_GET(self):
                path = self.path.split("?")[0]
                with web.lock:
                    if path == "/frame.jpg":
                        body, ctype = web.jpeg, "image/jpeg"
                    elif path == "/state.json":
                        body, ctype = json.dumps(web.state).encode(), "application/json"
                    elif path == "/":
                        body, ctype = PAGE.replace("__TOKEN__", web.token).replace("__RUN__", web.state["run"]).encode(), "text/html; charset=utf-8"
                    else:
                        self.send_error(404)
                        return
                self.send_response(200)
                self.send_header("Content-Type", ctype)
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def do_POST(self):
                ok, err = False, None
                try:
                    # The per-run token (only readable by our own page) is the real check. Origin
                    # varies by browser (absent, "null", localhost vs 127.0.0.1); only reject
                    # an explicit non-local site.
                    origin = self.headers.get("Origin", "")
                    host = urlsplit(origin).hostname if origin not in ("", "null") else None
                    if host not in (None, "127.0.0.1", "localhost", "::1"):
                        raise ValueError(f"bad origin {origin!r}")
                    n = int(self.headers.get("Content-Length", 0))
                    if self.path != "/action" or n > 1000:
                        raise ValueError("bad request")
                    d = json.loads(self.rfile.read(n))
                    if not secrets.compare_digest(str(d.get("token", "")), web.token):
                        raise ValueError("bad token")
                    if d.get("action") not in ("arm", "disarm", "open", "fist", "hold", "release"):
                        raise ValueError("bad action")
                    with web.lock:
                        web.actions.append(d["action"])
                    ok = True
                    print(f"[web] {d['action']}", flush=True)
                except Exception as e:
                    err = str(e)
                    print(f"[web] rejected request: {err}", flush=True)
                body = json.dumps({"ok": ok, "error": err}).encode()
                self.send_response(200 if ok else 403)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

        self.server = ThreadingHTTPServer(("127.0.0.1", port), H)
        self.server.daemon_threads = True
        threading.Thread(target=self.server.serve_forever, daemon=True).start()

    def pop_actions(self):
        with self.lock:
            a, self.actions = self.actions, []
        return a

    def update(self, jpeg=None, **state):
        with self.lock:
            if jpeg is not None:
                self.jpeg = jpeg
            self.state.update(state)

    def close(self):
        self.server.shutdown()
