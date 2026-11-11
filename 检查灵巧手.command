#!/bin/zsh
set -eu
cd "${0:A:h}"
open -a OrbStack
exec .venv/bin/python scripts/run_mac_hand.py --probe
