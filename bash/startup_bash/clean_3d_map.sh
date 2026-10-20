#!/bin/bash
# Re-clean a generated map using its fixed trajectory and cached local scans.
set -eo pipefail
WS="$HOME/sirius_jazzy_ws"
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
CLEANER_PYTHON="$HOME/.local/share/sirius/dufomap-venv/bin/python"
if [[ ! -x "$CLEANER_PYTHON" ]]; then CLEANER_PYTHON=python3; fi
"$CLEANER_PYTHON" "$WS/other_programs/theta_3d_map/clean_3d_map.py" "$@"
