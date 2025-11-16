# Sirius3 Description Package

Sirius3ロボットのROS 2記述パッケージです。

## 構成

```
sirius_description/
├── launch/
│   └── sim.launch.py          # シミュレーション起動用launchファイル
├── sdf/
│   ├── sirius3.sdf           # 元の統合SDFファイル
│   ├── sirius3_model.sdf     # ロボットモデルのみのSDFファイル
│   └── Sirius3.urdf          # URDFファイル
├── worlds/
│   └── sirius_world.sdf      # ワールドファイル
├── urdf/
│   └── meshes/               # メッシュファイル
└── setup_gazebo_env.sh       # Gazebo環境変数設定スクリプト
```

## 使用方法

### 1. ビルド
```bash
cd ~/sirius_jazzy_ws
colcon build --packages-select sirius_description
source install/setup.bash
```

### 2. シミュレーション起動
```bash
ros2 launch sirius_description sim.launch.py
```

### 3. 手動でのGazebo起動
```bash
# ワールドファイルを直接起動
gz sim ~/sirius_jazzy_ws/src/sirius_description/worlds/sirius_world.sdf
```

## 主な変更点

- 元の`sirius3.sdf`をワールド部分とロボット部分に分離
- `worlds/sirius_world.sdf`: 物理シミュレーション、照明、地面を含むワールド環境
- `sdf/sirius3_model.sdf`: Sirius3ロボットモデルのみ
- `launch/sim.launch.py`: ROS 2とGazebo間のブリッジを含む統合launchファイル

## 機能

- 差動二輪駆動制御
- Odometry配信
- TF配信
- テレオペレーション（キーボード制御）
- ROS 2 <-> Gazebo ブリッジ

## 制御

launchファイルを起動すると、自動的にテレオペレーションが起動します：
- `i`: 前進
- `j`: 左回転
- `l`: 右回転
- `k`: 停止
- `,`: 後退

## Package Structure

```
sirius_description/
├── CMakeLists.txt          # Build configuration
├── package.xml             # Package metadata and dependencies
├── README.md              # This file
├── config/                # Configuration files
├── launch/                # Launch files
│   └── display.launch.py  # Launch file for RViz visualization
├── rviz/                  # RViz configuration files
│   └── sirius_robot.rviz  # RViz configuration for Sirius robot
├── sdf/                   # Gazebo SDF files
│   └── sirius_robot.sdf   # Gazebo model definition
├── urdf/                  # URDF files
│   └── sirius_robot.urdf.xacro  # Robot description in URDF format
└── worlds/                # Gazebo world files
```

## Robot Specifications

- **Base dimensions**: 0.6m x 0.4m x 0.15m
- **Weight**: 2.0kg (base)
- **Wheel radius**: 0.12m
- **Wheel width**: 0.06m
- **Drive type**: Differential drive
- **Caster wheels**: Front and rear for stability

## Usage

### 1. Build the package

```bash
cd ~/sirius_jazzy_ws
colcon build --packages-select sirius_description
source install/setup.bash
```

### 2. Launch RViz visualization

```bash
ros2 launch sirius_description display.launch.py
```

### 3. Launch with GUI for joint control

```bash
ros2 launch sirius_description display.launch.py use_gui:=true
```

### 4. Launch for simulation

```bash
ros2 launch sirius_description display.launch.py use_sim_time:=true
```

## Launch Parameters

- `use_sim_time`: Set to 'true' when using with Gazebo simulation (default: 'false')
- `use_gui`: Set to 'true' to launch joint state publisher GUI (default: 'true')

## Robot Frame Structure

```
sirius/base_footprint
└── sirius/base_link
    ├── sirius/left_wheel
    ├── sirius/right_wheel
    ├── sirius/front_caster
    └── sirius/rear_caster
```

## Dependencies

- `urdf`
- `xacro`
- `robot_state_publisher`
- `rviz2`
- `joint_state_publisher`
- `joint_state_publisher_gui`
- `ros_gz_bridge`
- `tf2_ros`
- `teleop_twist_keyboard`
- `launch`
- `launch_ros`

## Notes

- The robot uses a differential drive configuration with two main wheels and two caster wheels for support
- Frame prefix is set to "sirius/" to avoid naming conflicts
- The robot description is compatible with both RViz2 visualization and Gazebo simulation


## TF Debugging Commands
To debug and visualize the TF frames, you can use the following commands:
1. View the TF frame tree:
   ```bash
   ros2 run tf2_tools view_frames
   ```
   This will generate a PDF file named `frames.pdf` in the current directory, showing the TF frame hierarchy.