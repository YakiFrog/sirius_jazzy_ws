#!/usr/bin/env bash
set -eo pipefail

APP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIRIUS_WS="$(cd "${APP_DIR}/../.." && pwd)"

source /opt/ros/jazzy/setup.bash
if [ -f "${SIRIUS_WS}/install/setup.bash" ]; then
    source "${SIRIUS_WS}/install/setup.bash"
fi
set -u

exec python3 "${APP_DIR}/rosbag_repair_app.py" "$@"
