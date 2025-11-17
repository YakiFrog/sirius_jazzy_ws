"""
Sirius ROS2 Launch Manager - UI Components
UIコンポーネントの定義
"""

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QGroupBox
)
from PySide6.QtCore import Qt
from PySide6.QtGui import QFont


class LaunchButtonUI(QWidget):
    """起動ボタンウィジェットのUI"""
    
    def __init__(self, name, description=""):
        super().__init__()
        self.name = name
        self.description = description
        self.setup_ui()
    
    def setup_ui(self):
        """UIのセットアップ"""
        layout = QHBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)
        
        # 起動ボタン
        self.launch_btn = QPushButton(f"▶ {self.name}")
        self.launch_btn.setMinimumWidth(200)
        layout.addWidget(self.launch_btn)
        
        # ステータス表示
        self.status_label = QLabel("●")
        self.status_label.setStyleSheet("color: gray; font-size: 20px;")
        self.status_label.setFixedWidth(30)
        self.status_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.status_label)
        
        # 説明ラベル
        desc_label = QLabel(self.description)
        desc_label.setStyleSheet("color: gray;")
        layout.addWidget(desc_label, 1)
        
        # 停止ボタン
        self.stop_btn = QPushButton("■ 停止")
        self.stop_btn.setMaximumWidth(100)
        self.stop_btn.setEnabled(False)
        layout.addWidget(self.stop_btn)
    
    def update_status(self, is_running):
        """ステータス表示を更新"""
        self.launch_btn.setEnabled(not is_running)
        self.stop_btn.setEnabled(is_running)
        
        if is_running:
            self.status_label.setStyleSheet("color: #28a745; font-size: 16px;")  # 緑
            self.status_label.setText("●")
        else:
            self.status_label.setStyleSheet("color: gray; font-size: 16px;")
            self.status_label.setText("●")


class MainWindowUI:
    """メインウィンドウのUIセットアップ"""
    
    @staticmethod
    def setup_ui(window):
        """UIのセットアップ"""
        from PySide6.QtWidgets import QWidget, QVBoxLayout, QScrollArea, QFrame
        
        window.setWindowTitle("Sirius ROS2 Launch Manager")
        window.setMinimumSize(800, 600)
        
        central_widget = QWidget()
        window.setCentralWidget(central_widget)
        
        main_layout = QVBoxLayout(central_widget)
        main_layout.setContentsMargins(10, 10, 10, 10)
        
        # タイトル
        title = QLabel("Sirius ROS2 Launch Manager")
        title_font = QFont()
        title_font.setPointSize(16)
        title_font.setBold(True)
        title.setFont(title_font)
        title.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(title)
        
        # 情報ラベル
        info_label = QLabel("ボタンを押すとTerminatorのタブで起動します (--new-tab使用) | 緑●=起動中")
        info_label.setStyleSheet("color: gray; font-style: italic;")
        info_label.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(info_label)
        
        # プリセットセクション
        preset_group = QGroupBox("プリセット")
        preset_group.setStyleSheet("QGroupBox { font-weight: bold; }")
        preset_layout = QHBoxLayout()
        preset_group.setLayout(preset_layout)
        main_layout.addWidget(preset_group)
        
        # スクロールエリア
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        
        scroll_content = QWidget()
        buttons_layout = QVBoxLayout(scroll_content)
        buttons_layout.setSpacing(5)
        
        scroll.setWidget(scroll_content)
        main_layout.addWidget(scroll)
        
        return preset_layout, buttons_layout
    
    @staticmethod
    def create_preset_button(preset_name):
        """プリセットボタンを作成"""
        preset_btn = QPushButton(f"▶ {preset_name}")
        preset_btn.setMinimumHeight(40)
        preset_btn.setStyleSheet("background-color: #007bff; color: white; font-weight: bold;")
        return preset_btn
    
    @staticmethod
    def create_group(title):
        """グループボックスを作成"""
        group = QGroupBox(title)
        group_layout = QVBoxLayout()
        group.setLayout(group_layout)
        return group, group_layout
