#!/bin/bash
set -eo pipefail
WS="$HOME/sirius_jazzy_ws"
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
export OPENBLAS_NUM_THREADS=2
export OMP_NUM_THREADS=2
python3 "$WS/other_programs/theta_3d_map/recolor_3d_map.py" "$@"
