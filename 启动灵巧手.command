#!/bin/zsh
set -eu
cd "${0:A:h}"
open -a OrbStack
# No motor action occurs on startup. Initialize and enable explicitly in the UI.
exec .venv/bin/python scripts/run_mac_hand.py --motor-only --no-render --allow-motion
