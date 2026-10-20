#!/bin/bash
# Usage: run_offline_3d_mapping.sh [bag_directory] [--known-single-floor] [--mode wheel] [-o output]
set -eo pipefail
WS="$HOME/sirius_jazzy_ws"
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
export OMP_NUM_THREADS="${OMP_NUM_THREADS:-2}"
export OPENBLAS_NUM_THREADS="${OPENBLAS_NUM_THREADS:-2}"
MAPPING_PYTHON="$HOME/.local/share/sirius/dufomap-venv/bin/python"
if [[ ! -x "$MAPPING_PYTHON" ]]; then MAPPING_PYTHON=python3; fi
"$MAPPING_PYTHON" "$WS/other_programs/theta_3d_map/offline_3d_mapping.py" "$@"
