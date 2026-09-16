#!/bin/bash
# THETA S (HDMI->USBキャプチャ) をROS2へ配信する。
# 起動時に送信fpsと解像度を選択できる（既定 fps=5, 1920x1080）。
# 引数でも指定可能: start_theta_capture.sh <fps> <解像度番号 or WxH>
#   例: start_theta_capture.sh 5 1      # fps5, 解像度メニュー1(1920x1080)
#   例: start_theta_capture.sh 5 1280x720
#
# 【重要】実効fpsの上限はキャプチャボード(I-O DATA HDPC-UT)のYUYVモードに律速される（PCではない）。
#   実測(I-O DATA HDPC-UT, YUYV): 1920x1080 -> 5 fps / 1280x720 -> 10 fps / 640x480 -> 30 fps
#   MJPGは1080p30を出すが、このボードはフレームが凍結(全フレーム同一・空白)するため使用不可。
#   よって1080pでは fps設定を30にしても約5fpsが天井。これが「30に見えない」理由。
#   最低720p運用（640x480は4:3で魚眼が縦長に歪むため使用しない）。滑らかさは1280x720(≤10fps)。
#
# 【向き補正】実機は各レンズが逆方向に90°回転（左=前CCW90°/右=後CW90°）。補正は投影時に
#   config/theta_calibration_real.yaml の image_roll_degrees(front=-90, back=+90) で行う
#   （画素を回すと近似中心/ROIで円がずれ残像が出るため、キャプチャ側では回さない）。
#
# fpsは間引きレート。範囲は 0.1〜30.0。録画は5推奨（RTABは2Hz取り込み）。
# 注意: 魚眼のキャリブ値(theta_calibration*.yaml)は1920x1080基準。変更時は中心/焦点距離を同スケールに。

WS_DIR="${HOME}/sirius_jazzy_ws"
source "$WS_DIR/install/setup.bash" 2>/dev/null || source /opt/ros/jazzy/setup.bash

# 検証済みの解像度と、そのYUYV実測上限fps（最低720p: 640x480は4:3で魚眼が縦長になるため除外）
RES_OPTIONS=("1920x1080" "1280x720")
RES_CAPS=("5" "10")
RES_LABELS=(
    "1920x1080  (既定・キャリブ基準, YUYV上限 約5fps)"
    "1280x720   (最小推奨, YUYV上限 約10fps)"
)

FPS="${1:-}"
RES_ARG="${2:-}"

if [ -z "$FPS" ]; then
    read -p "THETA送信fpsを入力してください (0.1〜30.0, 既定5): " FPS
    FPS="${FPS:-5}"
fi

# fpsの数値チェックと範囲clamp (0.1〜30.0)
if ! [[ "$FPS" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
    echo "fpsは数値で入力してください: '$FPS' (範囲 0.1〜30.0)" >&2
    exit 1
fi
if awk "BEGIN{exit !($FPS > 30)}"; then
    echo "fps上限30.0にclampします (指定:$FPS)" >&2
    FPS=30
fi
if awk "BEGIN{exit !($FPS < 0.1)}"; then
    echo "fps下限0.1にclampします (指定:$FPS)" >&2
    FPS=0.1
fi

# 解像度を決定（引数が番号ならメニュー選択、WxHならそのまま、無指定なら対話メニュー）
RES_CAP=""
if [[ "$RES_ARG" =~ ^[0-9]+$ ]]; then
    if [ "$RES_ARG" -ge 1 ] && [ "$RES_ARG" -le "${#RES_OPTIONS[@]}" ]; then
        RES="${RES_OPTIONS[$((RES_ARG - 1))]}"
        RES_CAP="${RES_CAPS[$((RES_ARG - 1))]}"
    else
        echo "解像度番号が範囲外です: '$RES_ARG' (1〜${#RES_OPTIONS[@]})" >&2
        exit 1
    fi
elif [ -n "$RES_ARG" ]; then
    RES="$RES_ARG"
else
    echo "解像度を選択してください:"
    for i in "${!RES_OPTIONS[@]}"; do
        echo "  $((i + 1))) ${RES_LABELS[$i]}"
    done
    read -p "番号を入力 (既定1): " RES_NUM
    RES_NUM="${RES_NUM:-1}"
    if ! [[ "$RES_NUM" =~ ^[0-9]+$ ]] || [ "$RES_NUM" -lt 1 ] || [ "$RES_NUM" -gt "${#RES_OPTIONS[@]}" ]; then
        echo "解像度番号が不正です: '$RES_NUM' (1〜${#RES_OPTIONS[@]})" >&2
        exit 1
    fi
    RES="${RES_OPTIONS[$((RES_NUM - 1))]}"
    RES_CAP="${RES_CAPS[$((RES_NUM - 1))]}"
fi

WIDTH="${RES%x*}"
HEIGHT="${RES#*x}"
if ! [[ "$WIDTH" =~ ^[0-9]+$ && "$HEIGHT" =~ ^[0-9]+$ ]]; then
    echo "解像度の書式が不正です: '$RES' (例: 1920x1080)" >&2
    exit 1
fi

# fpsはDOUBLE型のため整数入力は x.0 に正規化（fps:=5 だとINTEGER型エラーになる）
if [[ "$FPS" =~ ^[0-9]+$ ]]; then
    FPS="${FPS}.0"
fi

# デバイスはノード側で /dev/theta_capture が無ければ /dev/video0 にフォールバックする。
DEVICE="${THETA_DEVICE:-/dev/theta_capture}"
FOURCC="${THETA_FOURCC:-YUYV}"

# ボードの上限より高いfpsを指定しても実効は上限で頭打ちになる旨を注意
if [ -n "$RES_CAP" ] && [ "$FOURCC" = "YUYV" ] && awk "BEGIN{exit !($FPS > $RES_CAP)}"; then
    echo "注意: $RES のYUYV実測上限は約${RES_CAP}fps です。fps=$FPS を指定してもボード側で約${RES_CAP}fpsに制限されます。" >&2
fi

echo "THETA capture 起動: device=$DEVICE fourcc=$FOURCC ${WIDTH}x${HEIGHT} fps=$FPS"
exec ros2 run sirius_navigation theta_capture_node --ros-args \
    -p device:="$DEVICE" -p fourcc:="$FOURCC" \
    -p width:="$WIDTH" -p height:="$HEIGHT" -p fps:="$FPS"
