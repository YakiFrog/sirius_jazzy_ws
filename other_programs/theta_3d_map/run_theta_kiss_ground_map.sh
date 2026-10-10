#!/bin/bash
# KISS-ICPで地面XY投影の路面マップを生成（bagを対話選択）。実験的。
set -u
WS="$HOME/sirius_jazzy_ws"
source "$WS/install/setup.bash" 2>/dev/null || source /opt/ros/jazzy/setup.bash
mapfile -t BAGS < <(find "$HOME/rosbag2_data" /media/*/rosbag2_data -maxdepth 2 -name metadata.yaml -printf '%h\n' 2>/dev/null | sort -r | awk '!seen[$0]++')
if [ ${#BAGS[@]} -eq 0 ]; then echo "bagが見つかりません"; read -r -p "Enterで閉じます..." _; exit 1; fi
echo "路面マップを作るbagを選択:"
for i in "${!BAGS[@]}"; do printf "  [%d] %s\n" "$((i+1))" "$(basename "${BAGS[$i]}")"; done
read -r -p "番号 (既定1): " SEL; SEL=${SEL:-1}
BAG="${BAGS[$((SEL-1))]}"
read -r -p "LiDAR地上高[m] (既定 0.85): " GA; GA=${GA:-0.85}
python3 "$WS/other_programs/theta_3d_map/theta_kiss_ground_map.py" "$BAG" --out-prefix "$BAG/ground_map" --ground-abs "$GA" --subsample 1 --band 0.35 --cell 0.04 --max-range 60
echo "生成: $BAG/ground_map.png / .ply"
read -r -p "Enterで閉じます..." _
