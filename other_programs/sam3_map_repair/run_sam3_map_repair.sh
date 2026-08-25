#!/usr/bin/env bash
set -e

APP_DIR="$(cd "$(dirname "$0")" && pwd)"
source /opt/ros/jazzy/setup.bash
source "$HOME/sirius_jazzy_ws/install/setup.bash" 2>/dev/null || true
exec python3 "$APP_DIR/sam3_map_repair_app.py" "$@"
