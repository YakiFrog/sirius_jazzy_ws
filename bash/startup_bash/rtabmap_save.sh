#!/bin/bash
trap 'echo ""; echo "Ctrl + Cが押されましたが、ウィンドウは閉じません"' 2
cd ~/sirius_jazzy_ws

REQUESTED_MAP_NAME="${1:-}"
SLAM_BASE_YAML="${2:-}"
AUTO_NAME_MODE=false
if [ -n "$REQUESTED_MAP_NAME" ]; then
    AUTO_NAME_MODE=true
fi

make_unique_map_name() {
    local requested="$1"
    local sanitized
    local candidate
    local suffix=2

    sanitized=$(printf '%s' "$requested" | sed 's/[^A-Za-z0-9._-]/_/g')
    sanitized=${sanitized:-semantic_map_$(date +%Y%m%d_%H%M%S)}
    candidate="$sanitized"
    while [ -e "$HOME/sirius_jazzy_ws/maps_waypoints/maps/rtabmap_${candidate}" ]; do
        candidate=$(printf '%s_%02d' "$sanitized" "$suffix")
        suffix=$((suffix + 1))
    done
    printf '%s' "$candidate"
}

while : ;do
    echo "------------------------------------------------"
    echo "RTAB-Map 統合保存スクリプト (Grid/PLY/Color)"
    echo "------------------------------------------------"
    
    # ROS環境の読み込み
    if [ -f "install/setup.bash" ]; then
        source install/setup.bash
    else
        source /opt/ros/jazzy/setup.bash
    fi

    if [ "$AUTO_NAME_MODE" = true ]; then
        map_name=$(make_unique_map_name "$REQUESTED_MAP_NAME")
        echo "自動地図名: $map_name"
    else
        read -p "Press [Enter] key to start export process..."
        echo "保存するマップ名を入力してください (例: my_map): "
        read map_name

        if [ -z "$map_name" ]; then
            echo "エラー: マップ名が空です。"
            continue
        fi
    fi

    MAP_DIR="$HOME/sirius_jazzy_ws/maps_waypoints/maps/rtabmap_$map_name"
    mkdir -p "$MAP_DIR"
    
    # 定義
    PLY_OUT="$MAP_DIR/rtabmap_${map_name}.ply"

    # 1. 2Dグリッドマップの保存
    echo "[1/3] 2Dグリッドマップ (Nav2形式) を保存中..."
    ros2 run nav2_map_server map_saver_cli -f "$MAP_DIR/rtabmap_$map_name" --ros-args -r map:=/rtabmap/grid_map -p map_subscribe_transient_local:=true -p save_map_timeout:=10000.0
    
    # 2. 3D点群マップの保存 (トピック /rtabmap/cloud_map から直接保存)
    echo "[2/3] 3D点群マップ (PLY/Color) を保存中..."
    SAM3_PLY_OUT="$PLY_OUT" python3 << 'PYEOF'
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import struct, os, sys, math, numpy as np

class CloudToPly(Node):
    def __init__(self, output_path):
        super().__init__('cloud_to_ply_saver')
        from rclpy.parameter import Parameter
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.output_path = output_path
        
        from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
        qos_profile = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        # Try /cloud_map first, then /rtabmap/cloud_map if no publisher
        self.topic_name = '/cloud_map'
        self.subscription = self.create_subscription(PointCloud2, self.topic_name, self.listener_callback, qos_profile)
        self.get_logger().info(f'Waiting for {self.topic_name} to save to {self.output_path} (Timeout: 10s)...')
        
        # Use Steady Clock (Wall Time) for the timeout to avoid issues with Sim Time jumps
        from rclpy.clock import Clock, ClockType
        self.timer = self.create_timer(10.0, self.timeout_callback, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def timeout_callback(self):
        self.get_logger().error(f'Timeout: No message received on {self.topic_name} after 10 seconds.')
        sys.exit(1)

    def listener_callback(self, msg):
        self.get_logger().info(f'Received cloud: {msg.width * msg.height} points.')
        
        # Color field detection
        fields = {f.name: f for f in msg.fields}
        color_field = None
        for cf in ["rgb", "rgba", "colors"]:
            if cf in fields:
                color_field = cf
                break
        
        points = []
        try:
            # Efficiently read points
            read_fields = ("x", "y", "z")
            if color_field: read_fields += (color_field,)
            
            for p in pc2.read_points(msg, field_names=read_fields, skip_nans=True):
                x, y, z = p[0], p[1], p[2]
                if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)): continue
                
                r, g, b = 150, 150, 150 # Default gray
                if color_field:
                    val = p[3]
                    # Robust Color Decoding (handling float/int packed variants)
                    try:
                        if isinstance(val, (float, np.float32)):
                            packed = struct.unpack('I', struct.pack('f', val))[0]
                        else:
                            packed = int(val)
                        # RTAB-Map convention: usually BGRA or RGBA in a 32-bit int
                        # We try to extract R, G, B
                        r = (packed >> 16) & 0xFF
                        g = (packed >> 8) & 0xFF
                        b = packed & 0xFF
                    except: pass
                points.append((x, y, z, r, g, b))
        except Exception as e:
            self.get_logger().error(f"Error: {e}")
            sys.exit(1)

        if points:
            os.makedirs(os.path.dirname(self.output_path), exist_ok=True)
            with open(self.output_path, 'wb') as f:
                header = f"ply\nformat binary_little_endian 1.0\n"
                header += f"element vertex {len(points)}\n"
                header += "property float x\nproperty float y\nproperty float z\n"
                header += "property uchar red\nproperty uchar green\nproperty uchar blue\n"
                header += "end_header\n"
                f.write(header.encode())
                for (px, py, pz, pr, pg, pb) in points:
                    f.write(struct.pack('<fffBBB', px, py, pz, pr, pg, pb))
            print(f'SUCCESS: Exported {len(points)} points with colors.')
            sys.exit(0)
        else:
            sys.exit(1)

def main():
    rclpy.init()
    node = CloudToPly(os.environ.get('SAM3_PLY_OUT', '/tmp/map.ply'))
    try: rclpy.spin(node)
    except SystemExit: pass
    finally: rclpy.shutdown()

if __name__ == '__main__': main()
PYEOF

    if [ -f "$PLY_OUT" ]; then
        echo "SUCCESS: 3Dマップ (PLY) を保存しました: $PLY_OUT"
    else
        echo "ERROR: 3Dマップの保存に失敗しました。RTAB-Mapが停止しているか、局在化していない可能性があります。"
    fi

    # 3. SAM3 2Dセマンティック地図の保存とカラーPNG生成
    echo "[3/3] SAM3 2Dセマンティック地図を保存中..."
    ros2 topic pub --once /sam3/save_indexed_map std_msgs/msg/String "{data: '$MAP_DIR/rtabmap_${map_name}.colored'}" >/dev/null 2>&1
    sleep 1

    # カラー地図のレンダリング (PGM + PLY および Indexed Grid から PNG 生成)
    COLORIZER_SCRIPT="$HOME/sirius_jazzy_ws/src/sirius/sirius_navigation/sirius_navigation/sam3_map_colorizer.py"
    MAP_BASE_PATH="$MAP_DIR/rtabmap_$map_name"
    
    if [ -f "$COLORIZER_SCRIPT" ]; then
        python3 "$COLORIZER_SCRIPT" "$MAP_BASE_PATH"
    fi

    # Indexed Grid から完全なカラーPNGを自動生成
    python3 -c "
import cv2, json, os, numpy as np
pgm_p = '$MAP_DIR/rtabmap_${map_name}.colored.pgm'
json_p = '$MAP_DIR/rtabmap_${map_name}.colored.json'
out_p = '$MAP_DIR/rtabmap_${map_name}.semantic_full.png'
if os.path.exists(pgm_p) and os.path.exists(json_p):
    pgm = cv2.imread(pgm_p, cv2.IMREAD_GRAYSCALE)
    with open(json_p) as f: meta = json.load(f)
    palette = np.array(meta['palette'], dtype=np.uint8)
    rgb = np.zeros((pgm.shape[0], pgm.shape[1], 3), dtype=np.uint8)
    for i in range(len(palette)):
        rgb[pgm == i] = palette[i]
    cv2.imwrite(out_p, rgb)
    print(f'✓ 全領域セマンティックカラー画像を保存しました: {out_p}')
" 2>/dev/null

    if [ -n "$SLAM_BASE_YAML" ]; then
        REBASE_SCRIPT="$HOME/sirius_jazzy_ws/bash/startup_bash/rebase_semantic_map_to_slam.py"
        echo ""
        echo "[追加] SLAM ToolboxのPGMを構造ベースにした地図を生成中..."
        echo "  構造地図: $SLAM_BASE_YAML"
        if [ -f "$REBASE_SCRIPT" ]; then
            if ! python3 "$REBASE_SCRIPT" "$MAP_BASE_PATH" "$SLAM_BASE_YAML"; then
                echo "ERROR: SLAM Toolboxベース版の生成に失敗しました。"
                echo "RTAB-Mapベースの標準版は保存済みです: $MAP_DIR"
            fi
        else
            echo "ERROR: 変換スクリプトがありません: $REBASE_SCRIPT"
        fi
    fi

    echo "------------------------------------------------"
    echo "完了しました！"
    echo "保存先: $MAP_DIR"
    if [ "$AUTO_NAME_MODE" = true ]; then
        break
    fi
done
