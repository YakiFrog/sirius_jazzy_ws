#!/bin/bash
# Isolated offline FastLIO replay; raw bag is unchanged. Omit bag to select by number.
set -eo pipefail
WS="$HOME/sirius_jazzy_ws"
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
export OMP_NUM_THREADS="${OMP_NUM_THREADS:-2}"
export OPENBLAS_NUM_THREADS="${OPENBLAS_NUM_THREADS:-2}"
python3 "$WS/other_programs/theta_3d_map/recompute_fastlio.py" "$@"
