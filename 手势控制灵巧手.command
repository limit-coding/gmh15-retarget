#!/bin/zsh
# Double-click: opens iTerm2 (has camera permission) and runs the one-command launcher:
# OrbStack -> hand service -> position init if needed -> gesture control -> web page.
osascript -e 'tell application "iTerm" to tell current session of (create window with default profile) to write text "CONTROL_HZ=10 ~/灵巧手/scripts/start_all.sh"'
