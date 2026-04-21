#!/bin/bash
trap 'echo ""; echo "Ctrl + Cが押されましたが、ウィンドウは閉じません"' 2
cd ~/sirius_jazzy_ws
while : ;do
    echo "------------------------------------------------"
    echo "RTAB-Map 統合保存スクリプト (Grid/PLY/Color)"
    echo "------------------------------------------------"
    read -p "Press [Enter] key to start export process..."
    
    # ROS環境の読み込み
    if [ -f "install/setup.bash" ]; then
        source install/setup.bash
    else
        source /opt/ros/jazzy/setup.bash
    fi
    export ROS_DOMAIN_ID=56

    echo "保存するマップ名を入力してください (例: my_map): "
    read map_name
    
    if [ -z "$map_name" ]; then
        echo "エラー: マップ名が空です。"
        continue
    fi

    MAP_DIR="$HOME/sirius_jazzy_ws/maps_waypoints/maps"
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
        
        # Add a timeout timer to exit if no message arrives
        self.timer = self.create_timer(10.0, self.timeout_callback)

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

    # 3. カラー地図の生成 (ポストプロセシング)
    echo "[3/3] カラー地図を生成中 (PGM + PLY 統合)..."
    COLORIZER_SCRIPT="$HOME/sirius_jazzy_ws/src/sirius/sirius_navigation/sirius_navigation/sam3_map_colorizer.py"
    MAP_BASE_PATH="$MAP_DIR/rtabmap_$map_name"
    
    if [ -f "$COLORIZER_SCRIPT" ]; then
        python3 "$COLORIZER_SCRIPT" "$MAP_BASE_PATH"
    else
        echo "エラー: 着色スクリプトが見つかりません: $COLORIZER_SCRIPT"
    fi
    
    echo "------------------------------------------------"
    echo "完了しました。RVizなどのプロセスはCtrl+Cで停止しないでください。"
done
