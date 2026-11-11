#!/bin/zsh
# One command: OrbStack -> hand service -> position init (if needed) -> gesture retarget + web page.
# Run it in iTerm2 (it has camera permission). Ctrl+C or q in the camera window quits;
# the hand service is stopped too if this script started it.
#   CAMERA=1   Mac FaceTime camera in OpenCV numbering (0 is the palm camera)
set -u
ROOT="${0:A:h:h}"
cd "$ROOT" || exit 1
PY="$ROOT/.venv/bin/python"
CAMERA="${CAMERA:-1}"
CONTROL_HZ="${CONTROL_HZ:-30}"   # 30 = smooth mode (camera 30 fps); CONTROL_HZ=10 for the old mode
STAMP=$(date +%Y%m%d-%H%M%S)
started_service=0
svc_pid=""

state() { curl -s -m 1 http://127.0.0.1:8765/api/state 2>/dev/null; }

cleanup() {
  if [[ $started_service == 1 ]]; then
    echo "停止灵巧手服务…"
    [[ -n $svc_pid ]] && kill -INT $svc_pid 2>/dev/null
    docker stop -t 8 gmh15-mac >/dev/null 2>&1
  fi
}
trap cleanup EXIT INT TERM

# 1. OrbStack
if ! docker info >/dev/null 2>&1; then
  echo "启动 OrbStack…"
  open -a OrbStack
  for i in {1..60}; do docker info >/dev/null 2>&1 && break; sleep 1; done
  docker info >/dev/null 2>&1 || { echo "OrbStack 没有就绪"; exit 1; }
fi

# 2. hand service (reuse if already running)
if state | grep -q '"connected": true'; then
  echo "灵巧手服务已在运行，直接使用"
else
  echo "启动灵巧手服务…（日志 runs/mac_driver/server-$STAMP.log）"
  "$PY" scripts/run_mac_hand.py --motor-only --no-render --allow-motion --control-hz "$CONTROL_HZ" > "runs/mac_driver/server-$STAMP.log" 2>&1 &
  svc_pid=$!
  started_service=1
  for i in {1..45}; do state | grep -q '"connected": true' && break; sleep 1; done
  state | grep -q '"connected": true' || { echo "服务没有连上灵巧手，看日志："; tail -5 "runs/mac_driver/server-$STAMP.log"; exit 1; }
fi

# A pre-existing legacy service cannot become 30 Hz by changing only the client.
if [[ "$CONTROL_HZ" != 10 ]]; then
  actual_hz=$(curl -s -m 2 http://127.0.0.1:8765/api/teleop/config | "$PY" -c 'import json,sys; print(json.load(sys.stdin)["max_send_hz"])')
  if ! "$PY" -c 'import sys; sys.exit(0 if float(sys.argv[1]) == float(sys.argv[2]) else 1)' "$actual_hz" "$CONTROL_HZ"; then
    echo "当前服务为 $actual_hz Hz；请先关闭旧版灵巧手服务，再打开流畅版入口。"
    exit 1
  fi
fi

# 3. position init only if not done in this service session
if ! state | grep -q '"last_init_result": 1'; then
  echo "位置初始化（手指会回零，约 2 秒）…"
  "$PY" -c "
import sys; sys.path.insert(0, '.')
from hand_client import Hand
h = Hand('right'); print('初始化完成', h.initialize()); h.stop()" || { echo "初始化失败"; exit 1; }
fi

# 4. open the page once the gesture program serves it
( for i in {1..40}; do curl -s -m 1 http://127.0.0.1:8766/state.json >/dev/null 2>&1 && { open http://127.0.0.1:8766/; break; }; sleep 1; done ) &

echo "启动手势控制（摄像头 $CAMERA，未使能；在网页上点「使能真机」）…"
echo "录制：runs/hw_$STAMP.jsonl"
vision_args=()
if [[ "$CONTROL_HZ" == 30 ]]; then
  vision_args=(--camera-fps 30 --preview-hz 10 --tracking-width 640 --tracking-hands 1 --latest-frame --sim-size 210)
fi
"$PY" scripts/run_sim.py --camera "$CAMERA" --side right --hardware --i-have-checked-safety \
  --url http://127.0.0.1:8765 --send-hz "$CONTROL_HZ" "${vision_args[@]}" --record "runs/hw_$STAMP.jsonl"
