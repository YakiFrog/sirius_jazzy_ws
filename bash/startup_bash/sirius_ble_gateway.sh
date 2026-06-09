#!/usr/bin/env bash
set -euo pipefail

choice="${1:-}"
battery_mac=""

if [[ -z "${choice}" ]]; then
  echo "バッテリーの接続先を選んでください:"
  echo "  1) 1台目 (7E:97:24)"
  echo "  2) 2台目 (57:90:E8)"
  echo "  0) RemoteControllerのみ起動"
  read -r -p "選択 [0-2] > " choice
fi

case "${choice}" in
  1|"1台目"|"battery1")
    battery_mac="F4:9D:8A:7E:97:24"
    ;;
  2|"2台目"|"battery2")
    battery_mac="F4:9D:8A:57:90:E8"
    ;;
  0|"")
    battery_mac=""
    ;;
  *)
    battery_mac="${choice}"
    ;;
esac

cd ~/sirius_jazzy_ws
if [[ -x "${HOME}/sirius_face_anim2/venv/bin/python" ]]; then
  venv_site_packages="$("${HOME}/sirius_face_anim2/venv/bin/python" -c 'import site; print(site.getsitepackages()[0])')"
  export PYTHONPATH="${venv_site_packages}:${PYTHONPATH:-}"
fi

set +u
source install/setup.bash
set -u

if [[ -n "${battery_mac}" ]]; then
  echo "BLE管理ノード起動: RemoteController + Battery(${battery_mac})"
  ros2 launch sirius_navigation sirius_ble_gateway.launch.py \
    enable_remote_server:=true \
    enable_battery_client:=true \
    battery_mac:="${battery_mac}" \
    battery_scan_before_connect:=false \
    publish_face_battery_params:=true
else
  echo "BLE管理ノード起動: RemoteControllerのみ"
  ros2 launch sirius_navigation sirius_ble_gateway.launch.py \
    enable_remote_server:=true \
    enable_battery_client:=false \
    battery_scan_before_connect:=false \
    publish_face_battery_params:=true
fi
