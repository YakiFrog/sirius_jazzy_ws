#!/bin/bash
trap 'echo ""; echo "Ctrl + Cが押されましたが、ウィンドウは閉じません"' 2
cd ~/sirius_jazzy_ws

MAPS_DIR="${HOME}/sirius_jazzy_ws/maps_waypoints/maps"
LANDMARKS_DIR="${HOME}/sirius_jazzy_ws/maps_waypoints/landmarks"
CURRENT_MAP_STATE="${HOME}/.sirius_nav2_current_map.yaml"

# 地図ファイル一覧を取得
get_map_list() {
    find "$MAPS_DIR" -name "*.yaml" -type f | sort
}

# 地図を選択
select_map() {
    echo "========================================="
    echo "  利用可能な地図一覧"
    echo "========================================="
    
    maps=($(get_map_list))
    
    if [ ${#maps[@]} -eq 0 ]; then
        echo "エラー: $MAPS_DIR に地図ファイルが見つかりません"
        return 1
    fi
    
    for i in "${!maps[@]}"; do
        map_name=$(basename "${maps[$i]}" .yaml)
        echo "  [$((i+1))] $map_name"
    done
    
    echo "========================================="
    echo -n "番号を入力してください (1-${#maps[@]}): "
    read selection
    
    if [[ ! "$selection" =~ ^[0-9]+$ ]] || [ "$selection" -lt 1 ] || [ "$selection" -gt ${#maps[@]} ]; then
        echo "エラー: 無効な選択です"
        return 1
    fi
    
    selected_map="${maps[$((selection-1))]}"
    echo "選択された地図: $(basename "$selected_map" .yaml)"
    echo ""
}

write_current_map_state() {
    local map_path="$1"
    python3 - "$map_path" "$LANDMARKS_DIR" "$CURRENT_MAP_STATE" <<'PY'
import os
import sys
import yaml

map_path, landmarks_dir, state_path = sys.argv[1:4]
map_yaml = os.path.basename(map_path)
map_name = os.path.splitext(map_yaml)[0]
matched_landmark = ""

if os.path.isdir(landmarks_dir):
    for filename in sorted(os.listdir(landmarks_dir)):
        if not filename.endswith((".yaml", ".yml")):
            continue
        path = os.path.join(landmarks_dir, filename)
        try:
            with open(path, "r") as f:
                data = yaml.safe_load(f) or {}
        except Exception:
            continue
        map_info = data.get("map", {})
        candidates = {
            str(map_info.get("yaml", "")),
            str(map_info.get("name", "")),
            os.path.basename(str(map_info.get("path", ""))),
        }
        if map_yaml in candidates or map_name in candidates:
            matched_landmark = path
            break

state = {
    "current_map": map_path,
    "current_map_name": map_name,
    "current_map_yaml": map_yaml,
    "current_landmarks": matched_landmark,
}
with open(state_path, "w") as f:
    yaml.safe_dump(state, f, allow_unicode=True, sort_keys=False)

if matched_landmark:
    print(f"対応ランドマーク: {os.path.basename(matched_landmark)}")
else:
    print("対応ランドマーク: 未設定")
PY
}

while : ;do
    read -p "Press [Enter] key to start nav2 bringup..."
    source install/setup.bash
    
    selected_map=""
    select_map
    
    if [ $? -ne 0 ] || [ -z "$selected_map" ]; then
        echo "地図の選択がキャンセルされました"
        continue
    fi

    write_current_map_state "$selected_map"
    
    ros2 launch nav2_bringup bringup_launch.py \
    use_sim_time:=False \
    map:=$selected_map \
    params_file:=${HOME}/sirius_jazzy_ws/params/nav2_params.yaml \
    use_composition:=False
done
