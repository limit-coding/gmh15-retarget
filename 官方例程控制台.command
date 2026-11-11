#!/bin/zsh
set -eu
cd "${0:A:h}"
open -a OrbStack
for attempt in {1..30}; do
  if docker info >/dev/null 2>&1; then
    exec .venv/bin/python scripts/run_examples_web.py --open
  fi
  sleep 1
done
echo '本机 Linux 环境没有就绪，请打开 OrbStack 后重试。'
read -r '?按回车退出'
