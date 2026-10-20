#!/bin/bash
# Isolated optional native dependency; leave ROS/system Python packages intact.
set -euo pipefail
CLEANER_VENV="$HOME/.local/share/sirius/dufomap-venv"
python3 -m venv --system-site-packages "$CLEANER_VENV"
"$CLEANER_VENV/bin/python" -m pip install --no-deps 'dufomap==1.1.1'
"$CLEANER_VENV/bin/python" -c 'from dufomap import dufomap; print("DUFOMap ready")'
