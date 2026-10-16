#!/bin/bash
# Usage: run_offline_3d_mapping.sh [bag_directory] [--mode wheel] [-o output]
set -eo pipefail
WS="$HOME/sirius_jazzy_ws"
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
export OMP_NUM_THREADS="${OMP_NUM_THREADS:-2}"
export OPENBLAS_NUM_THREADS="${OPENBLAS_NUM_THREADS:-2}"
python3 "$WS/other_programs/theta_3d_map/offline_3d_mapping.py" "$@"
