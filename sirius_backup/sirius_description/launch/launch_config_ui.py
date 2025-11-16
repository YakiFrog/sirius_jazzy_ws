#!/usr/bin/env python3
"""
Sirius シミュレーション起動設定UI
PySide6を使用したGUIで起動パラメータを設定
"""

import os
import sys
from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                               QHBoxLayout, QCheckBox, QPushButton, QComboBox, 
                               QLabel, QGroupBox, QTextEdit)
from PySide6.QtCore import Qt, QEvent
from ament_index_python.packages import get_package_share_directory


class LaunchConfigUI(QMainWindow):
    """シミュレーション起動設定用のGUIウィンドウ"""
    
    def __init__(self):
        super().__init__()
        self.config = {
            'world_file': 'sirius_world.sdf',
            'spawn_robot': True,
            'robot_state_publisher': True,
            'tf_bridge': False,  # EKF使用時はFalse（IMU融合TFを使用）
            'odom_bridge': True,
            'twist_bridge': True,
            'joint_state_bridge': True,
            'lidar_bridge': True,
            'lidar2_bridge': True,
            'imu_bridge': True,
            'velodyne_bridge': True,
            'clock_bridge': True,
            'teleop': True,
            'rviz': True
        }
        self.launched = False
        self.init_ui()
        
    def init_ui(self):
        """UIの初期化"""
        self.setWindowTitle('Sirius シミュレーション起動設定')
        self.setGeometry(100, 100, 550, 700)
        
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)
        
        # ワールド選択セクション
        layout.addWidget(self._create_world_selection_group())
        
        # コンポーネント選択セクション
        layout.addWidget(self._create_components_group())
        
        # 説明表示エリア
        layout.addWidget(self._create_description_area())
        
        # 起動ボタン
        layout.addLayout(self._create_launch_button())
        
    def _create_world_selection_group(self):
        """ワールド選択グループの作成"""
        world_group = QGroupBox("ワールド選択")
        world_layout = QVBoxLayout()
        
        world_label = QLabel("起動するワールドファイル:")
        world_layout.addWidget(world_label)
        
        self.world_combo = QComboBox()
        self.world_combo.description = "シミュレーション環境（ワールド）のSDFファイルを選択します"
        self.world_combo.installEventFilter(self)
        
        # worldsディレクトリからSDFファイルを検索（srcディレクトリを直接参照）
        # このファイルのパスから推測: launch_config_ui.py -> launch/ -> sirius_description/ -> worlds/
        launch_file_dir = os.path.dirname(os.path.abspath(__file__))
        src_pkg_path = os.path.dirname(launch_file_dir)  # sirius_descriptionディレクトリ
        worlds_path = os.path.join(src_pkg_path, 'worlds')
        
        if os.path.exists(worlds_path):
            sdf_files = sorted([f for f in os.listdir(worlds_path) if f.endswith('.sdf')])
            if sdf_files:
                self.world_combo.addItems(sdf_files)
            else:
                self.world_combo.addItem('sirius_world.sdf')
        else:
            self.world_combo.addItem('sirius_world.sdf')
        
        world_layout.addWidget(self.world_combo)
        world_group.setLayout(world_layout)
        
        return world_group
    
    def _create_components_group(self):
        """コンポーネント選択グループの作成"""
        components_group = QGroupBox("起動するコンポーネント")
        components_layout = QVBoxLayout()
        
        self.checkboxes = {}
        
        # ロボット関連
        components_layout.addWidget(self._create_section_label("ロボット:"))
        self._add_checkbox(
            components_layout, 
            'spawn_robot', 
            "ロボットをスポーン", 
            True,
            "Gazeboシミュレーション環境内にSirius3ロボットを配置します"
        )
        self._add_checkbox(
            components_layout, 
            'robot_state_publisher', 
            "Robot State Publisher", 
            True,
            "ロボットの各リンク間の座標変換（TF）を配信します。RVizでの可視化に必要です"
        )
        
        # ブリッジ関連
        components_layout.addWidget(self._create_section_label("\nROS-Gazebo ブリッジ:"))
        self._add_checkbox(
            components_layout, 
            'tf_bridge', 
            "TF Bridge（⚠️ EKF使用時は無効化推奨）", 
            False,  # デフォルトはFalse
            "GazeboからROS 2へ座標変換（TF: odom→base_footprint）をブリッジします。\n"
            "⚠️ EKFでセンサフュージョンを使用する場合は無効化してください（TF競合を防ぐため）。\n"
            "EKFがIMU融合後のTFを配信します。"
        )
        self._add_checkbox(
            components_layout, 
            'odom_bridge', 
            "Odometry Bridge", 
            True,
            "ロボットの位置・速度情報（オドメトリ）をGazeboからROS 2へブリッジします。\n"
            "EKFの入力データとして使用されます。"
        )
        self._add_checkbox(
            components_layout, 
            'twist_bridge', 
            "Twist Bridge (cmd_vel)", 
            True,
            "ROS 2からGazeboへ速度指令（cmd_vel）をブリッジします。ロボット制御に必要です"
        )
        self._add_checkbox(
            components_layout, 
            'joint_state_bridge', 
            "Joint State Bridge", 
            True,
            "ロボットの関節（車輪など）の状態をGazeboからROS 2へブリッジします"
        )
        self._add_checkbox(
            components_layout, 
            'clock_bridge', 
            "Clock Bridge", 
            True,
            "シミュレーション時刻をGazeboからROS 2へブリッジします。時刻同期に必須です"
        )
        
        # センサー関連
        components_layout.addWidget(self._create_section_label("\nセンサー:"))
        self._add_checkbox(
            components_layout, 
            'lidar_bridge', 
            "Velodyne Bridge (/scan)", 
            True,
            "2D LiDARセンサーのデータ（/scan）をGazeboからROS 2へブリッジします。"
        )
        self._add_checkbox(
            components_layout, 
            'lidar2_bridge', 
            "Hokuyo Bridge (/hokuyo_scan)", 
            True,
            "2台目の2D LiDARセンサーのデータ（/hokuyo_scan）をブリッジします"
        )
        self._add_checkbox(
            components_layout, 
            'imu_bridge', 
            "IMU Bridge", 
            True,
            "IMU（慣性計測装置）のデータをブリッジします。姿勢・加速度・角速度情報を取得。\n"
            "EKFと組み合わせてオドメトリと融合することで、より正確な自己位置推定が可能です。"
        )
        self._add_checkbox(
            components_layout, 
            'velodyne_bridge', 
            "Velodyne Bridge (/velodyne_points)", 
            True,
            "3D LiDAR（Velodyne）の点群データをブリッジします。"
        )
        self._add_checkbox(
            components_layout, 
            'realsense_bridge', 
            "RealSense Camera Bridge", 
            True,
            "RGBDカメラ（RealSense）のRGB画像、深度画像、点群、カメラ情報をブリッジします"
        )
        
        # ツール関連
        components_layout.addWidget(self._create_section_label("\nツール:"))
        self._add_checkbox(
            components_layout, 
            'teleop', 
            "Teleopキーボードコントロール", 
            True,
            "キーボードでロボットを手動操作できるターミナルを起動します"
        )
        self._add_checkbox(
            components_layout, 
            'rviz', 
            "RViz2", 
            True,
            "ROS 2の可視化ツールを起動します。センサーデータやロボットの状態を確認できます"
        )
        
        components_group.setLayout(components_layout)
        return components_group
    
    def _create_section_label(self, text):
        """セクションラベルの作成"""
        label = QLabel(text)
        label.setStyleSheet("font-weight: bold;")
        return label
    
    def _create_description_area(self):
        """説明表示エリアの作成"""
        description_group = QGroupBox("説明")
        description_layout = QVBoxLayout()
        
        self.description_text = QTextEdit()
        self.description_text.setReadOnly(True)
        self.description_text.setMaximumHeight(80)
        self.description_text.setStyleSheet("""
            QTextEdit {
                background-color: #2b2b2b;
                color: #e0e0e0;
                border: 1px solid #555;
                border-radius: 3px;
                padding: 5px;
                font-size: 12px;
            }
        """)
        self.description_text.setText("項目の上にマウスを置くと、ここに説明が表示されます。")
        
        description_layout.addWidget(self.description_text)
        description_group.setLayout(description_layout)
        
        return description_group
    
    def _add_checkbox(self, layout, key, text, checked, description=None):
        """チェックボックスを追加"""
        checkbox = QCheckBox(text)
        checkbox.setChecked(checked)
        if description:
            checkbox.description = description
            checkbox.installEventFilter(self)
        self.checkboxes[key] = checkbox
        layout.addWidget(checkbox)
    
    def eventFilter(self, obj, event):
        """イベントフィルター：ホバー時に説明を表示"""
        if event.type() == QEvent.Type.Enter:
            if hasattr(obj, 'description'):
                self.description_text.setText(obj.description)
        elif event.type() == QEvent.Type.Leave:
            self.description_text.setText("項目の上にマウスを置くと、ここに説明が表示されます。")
        return super().eventFilter(obj, event)
    
    def _create_launch_button(self):
        """起動ボタンレイアウトの作成"""
        button_layout = QHBoxLayout()
        
        self.launch_button = QPushButton("起動")
        self.launch_button.description = "選択した設定でシミュレーションを開始します"
        self.launch_button.installEventFilter(self)
        self.launch_button.setStyleSheet("""
            QPushButton {
                background-color: #4CAF50;
                color: white;
                font-size: 16px;
                font-weight: bold;
                padding: 10px;
                border-radius: 5px;
            }
            QPushButton:hover {
                background-color: #45a049;
            }
        """)
        self.launch_button.clicked.connect(self.on_launch)
        button_layout.addWidget(self.launch_button)
        
        return button_layout
    
    def on_launch(self):
        """起動ボタンがクリックされた時の処理"""
        # 設定を保存
        self.config['world_file'] = self.world_combo.currentText()
        for key, checkbox in self.checkboxes.items():
            self.config[key] = checkbox.isChecked()
        
        self.launched = True
        self.close()
    
    def get_config(self):
        """設定を取得（起動された場合のみ）"""
        return self.config if self.launched else None


def show_launch_config_ui():
    """
    UIを表示して設定を取得する関数
    
    Returns:
        dict or None: ユーザーが選択した設定、またはキャンセルされた場合はNone
    """
    app = QApplication.instance()
    if app is None:
        app = QApplication(sys.argv)
    
    ui = LaunchConfigUI()
    ui.show()
    app.exec()
    
    return ui.get_config()
