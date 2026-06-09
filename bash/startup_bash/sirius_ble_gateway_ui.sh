#!/usr/bin/env bash
set -euo pipefail

cd ~/sirius_jazzy_ws

if [[ -x "${HOME}/sirius_face_anim2/venv/bin/python" ]]; then
  venv_site_packages="$("${HOME}/sirius_face_anim2/venv/bin/python" -c 'import site; print(site.getsitepackages()[0])')"
  export PYTHONPATH="${venv_site_packages}:${PYTHONPATH:-}"
fi

set +u
source install/setup.bash
set -u

ros2 run sirius_navigation sirius_ble_gateway_ui
