#!/bin/bash
trap 'echo ""; echo "Ctrl + Cが押されましたが、ウィンドウは閉じません"' 2
cd ~/sirius_jazzy_ws
while : ;do
    echo "------------------------------------------------"
    echo "RTAB-Map 保存スクリプト"
    echo "------------------------------------------------"
    read -p "Press [Enter] key to start RTAB-Map save..."
    
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

    # 1. 2Dグリッドマップの保存 (/rtabmap/grid_map を指定)
    echo "[1/2] 2Dグリッドマップを保存中..."
    ros2 run nav2_map_server map_saver_cli -f "$MAP_DIR/rtabmap_$map_name" --ros-args -r map:=/rtabmap/grid_map
    
        # 3D点群マップの保存 (トピック /rtabmap/cloud_map から直接保存)
        # これにより、RTAB-Mapによって最適化・フィルタリングされた「完成版」の点群が得られる
        echo "[2/2] 3D点群マップ (PLY) をトピックから保存中..."
        SAM3_PLY_OUT="$PLY_OUT" python3 << 'PYEOF'
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import struct, os, sys, math

class CloudToPly(Node):
    def __init__(self, output_path):
        super().__init__('cloud_to_ply_saver')
        # シミュレーション時間対応
        from rclpy.parameter import Parameter
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        
        self.output_path = output_path
        
        # QoS設定: RTAB-Mapの地図トピックに合わせて Transient Local / Reliable にする
        from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
        qos_profile = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.subscription = self.create_subscription(
            PointCloud2,
            '/cloud_map',
            self.listener_callback,
            qos_profile)
        self.get_logger().info('Waiting for /cloud_map topic (Transient Local)...')

    def listener_callback(self, msg):
        self.get_logger().info(f'Received cloud with {msg.width * msg.height} points.')
        
        points = []
        # フィールド: x, y, z, rgb (または rgba) を取得
        fields = [f.name for f in msg.fields]
        color_field = "rgb" if "rgb" in fields else ("rgba" if "rgba" in fields else None)
        
        if not color_field:
            self.get_logger().warn("No color field (rgb/rgba) found in point cloud.")

        try:
            for p in pc2.read_points(msg, field_names=("x", "y", "z", color_field) if color_field else ("x", "y", "z"), skip_nans=True):
                x, y, z = p[0], p[1], p[2]
                
                # 座標が有効か最終チェック
                if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                    continue

                # 色データのデコード
                r, g, b = 127, 127, 127 # デフォルト
                if color_field:
                    rgb_val = p[3]
                    if rgb_val is None or not math.isfinite(rgb_val):
                        continue
                        
                    # PointCloud2のrgbはfloatとしてパックされている場合がある
                    if isinstance(rgb_val, float):
                        packed = struct.unpack('I', struct.pack('f', rgb_val))[0]
                    else:
                        packed = int(rgb_val)
                    
                    r = (packed >> 16) & 0xFF
                    g = (packed >> 8) & 0xFF
                    b = packed & 0xFF
                
                points.append((x, y, z, r, g, b))
        except Exception as e:
            self.get_logger().error(f"Error parsing points: {e}")
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
            print(f'SUCCESS: Exported {len(points)} optimized points to {self.output_path}')
        else:
            print('ERROR: Resulting point cloud is empty.')
            sys.exit(1)
            
        # キャプチャしたら終了
        sys.exit(0)

def main():
    rclpy.init()
    ply_out = os.environ.get('SAM3_PLY_OUT', '/tmp/cloud.ply')
    node = CloudToPly(ply_out)
    try:
        # トピックが来るまで最大30秒待機
        rclpy.spin(node)
    except SystemExit:
        pass
    except Exception as e:
        print(f"Node execution error: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
PYEOF

    if [ -f "$PLY_OUT" ]; then
        echo "SUCCESS: 3Dマップを保存しました: $PLY_OUT"
    else
        echo "WARNING: 3Dマップの書き出しに失敗しました。"
    fi
    
    echo "------------------------------------------------"
    echo "保存プロセスが完了しました。"
done
