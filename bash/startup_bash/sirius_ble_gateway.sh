#!/usr/bin/env bash
set -euo pipefail

choice="${1:-}"
battery_mac=""
enable_remote=false
enable_ear=true
enable_face=true
enable_speech=true
# Battery and ear LEDs are enabled by default; remote BLE requires --remote.
for option in "${@:2}"; do
  case "${option}" in
    --remote) enable_remote=true ;;
    --ear-led) enable_ear=true ;;
    --all) enable_remote=true; enable_ear=true ;;
    --battery-only)
      enable_remote=false; enable_ear=false; enable_face=false; enable_speech=false ;;
    *) echo "不明なオプション: ${option}" >&2; exit 2 ;;
  esac
done

if [[ -z "${choice}" ]]; then
  echo "バッテリーの接続先を選んでください:"
  echo "  1) 1台目 (7E:97:24)"
  echo "  2) 2台目 (57:90:E8)"
  echo "  3) 3台目・実機用 (76:E9:58)"
  echo "  0) バッテリーなし (--remote / --ear-led 指定時)"
  read -r -p "選択 [0-3] > " choice
fi

case "${choice}" in
  1|"1台目"|"battery1")
    battery_mac="F4:9D:8A:7E:97:24"
    ;;
  2|"2台目"|"battery2")
    battery_mac="F4:9D:8A:57:90:E8"
    ;;
  3|"3台目"|"battery3")
    battery_mac="7C:E9:13:76:E9:58"
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
  battery_plaintext=false
  case "${battery_mac^^}" in
    F4:9D:8A:7E:97:24|F4:9D:8A:57:90:E8) battery_plaintext=true ;;
  esac
  echo "BLE管理ノード起動: Battery(${battery_mac}), RemoteController=${enable_remote}, Ear LED=${enable_ear}"
  ros2 launch sirius_navigation sirius_ble_gateway.launch.py \
    enable_remote_server:="${enable_remote}" \
    enable_battery_client:=true \
    enable_ear_led_client:="${enable_ear}" \
    battery_mac:="${battery_mac}" \
    battery_plaintext:="${battery_plaintext}" \
    battery_scan_before_connect:=true \
    publish_face_battery_params:="${enable_face}" \
    enable_battery_speech:="${enable_speech}"
else
  if [[ "${enable_remote}" == false && "${enable_ear}" == false ]]; then
    echo "起動するBLE機能がありません。バッテリーを選ぶか、--remote / --ear-led / --all を指定してください。" >&2
    exit 2
  fi
  echo "BLE管理ノード起動: RemoteController=${enable_remote}, Ear LED=${enable_ear}"
  ros2 launch sirius_navigation sirius_ble_gateway.launch.py \
    enable_remote_server:="${enable_remote}" \
    enable_battery_client:=false \
    enable_ear_led_client:="${enable_ear}" \
    battery_scan_before_connect:=false \
    publish_face_battery_params:=true
fi
