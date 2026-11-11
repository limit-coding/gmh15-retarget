#!/bin/zsh
set -eu
cd "${0:A:h}"
exec osascript -e 'tell application "iTerm" to tell current session of (create window with default profile) to write text "CONTROL_HZ=30 ~/灵巧手/scripts/start_all.sh"'
