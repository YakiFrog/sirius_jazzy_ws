"""
Sirius ROS2 Launch Manager - UI Components
UIコンポーネントの定義
"""

from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QGroupBox, QSizePolicy
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
        self.desc_label = QLabel(self.description)
        self.desc_label.setStyleSheet("color: gray;")
        layout.addWidget(self.desc_label, 1)

        # 停止ボタン
        self.stop_btn = QPushButton("■ 停止")
        self.stop_btn.setMaximumWidth(100)
        self.stop_btn.setEnabled(False)
        layout.addWidget(self.stop_btn)
    
    def update_status(self, is_running, has_error=False):
        """ステータス表示を更新"""
        self.launch_btn.setEnabled(True)  # 起動中も有効にし、フォーカスボタンとして機能させる
        self.stop_btn.setEnabled(is_running)
        
        if is_running:
            self.launch_btn.setText(f"👁 {self.name}")
            self.launch_btn.setStyleSheet("background-color: #17a2b8; color: white; font-weight: bold;")
        else:
            self.launch_btn.setText(f"▶ {self.name}")
            self.launch_btn.setStyleSheet("")
            
        if has_error:
            self.status_label.setStyleSheet("color: #dc3545; font-size: 16px; font-weight: bold;") # 赤
            self.status_label.setText("!")
        elif is_running:
            self.status_label.setStyleSheet("color: #28a745; font-size: 16px;")  # 緑
            self.status_label.setText("●")
        else:
            self.status_label.setStyleSheet("color: gray; font-size: 16px;")
            self.status_label.setText("●")


class CollapsibleSection(QWidget):
    """折りたたみ可能な小見出しセクション（既定: 折りたたみ）。

    見出し(ヘッダ)をクリックすると中身の表示/非表示を切り替える。
    中身は content_layout に追加する。
    """

    def __init__(self, title, expanded=False):
        super().__init__()
        self._title = title
        self._expanded = expanded

        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(2)

        header_row = QHBoxLayout()
        header_row.setContentsMargins(0, 0, 0, 0)
        header_row.setSpacing(4)

        self.header = QPushButton()
        self.header.setCheckable(True)
        self.header.setChecked(expanded)
        self.header.setCursor(Qt.PointingHandCursor)
        self.header.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.header.setStyleSheet(
            "QPushButton { text-align: left; color: #0b5ed7; font-weight: bold;"
            " font-size: 13px; border: none; border-bottom: 2px solid #cfe2ff;"
            " padding: 6px 2px 3px 2px; background: transparent; }"
            "QPushButton:hover { background-color: #eef5ff; }"
        )
        self.header.clicked.connect(self.toggle)
        header_row.addWidget(self.header, 1)

        # このセクション内で起動中の数（赤バッジ）
        self.badge = QLabel("")
        self.badge.setAlignment(Qt.AlignCenter)
        self.badge.setFixedSize(20, 20)
        self.badge.setStyleSheet(
            "QLabel { background-color: #dc3545; color: white; font-weight: bold;"
            " border-radius: 10px; font-size: 11px; }"
        )
        self.badge.setVisible(False)
        header_row.addWidget(self.badge)
        outer.addLayout(header_row)

        self.content = QWidget()
        self.content_layout = QVBoxLayout(self.content)
        self.content_layout.setContentsMargins(10, 2, 2, 2)
        self.content_layout.setSpacing(2)
        self.content.setVisible(expanded)
        outer.addWidget(self.content)

        self._update_header()

    def _update_header(self):
        arrow = "▼" if self._expanded else "▶"
        self.header.setText(f"{arrow}  {self._title}")

    def toggle(self):
        self.set_expanded(self.header.isChecked())

    def set_expanded(self, expanded):
        self._expanded = expanded
        self.header.setChecked(expanded)
        self.content.setVisible(expanded)
        self._update_header()

    def add_widget(self, widget):
        self.content_layout.addWidget(widget)

    def set_badge(self, count):
        """このセクションの起動中件数を表示（0は非表示）"""
        if count > 0:
            self.badge.setText(str(count))
            self.badge.setVisible(True)
        else:
            self.badge.setText("")
            self.badge.setVisible(False)


class MainWindowUI:
    """メインウィンドウのUIセットアップ"""
    
    @staticmethod
    def setup_ui(window, tab_names=None):
        """UIのセットアップ（タブ切り替え対応）"""
        from PySide6.QtWidgets import (QWidget, QVBoxLayout, QScrollArea, QFrame,
                                       QTabWidget, QTabBar, QSplitter, QSpinBox)

        window.setWindowTitle("Sirius ROS2 Launch Manager")
        window.setMinimumSize(900, 600)

        central_widget = QWidget()
        window.setCentralWidget(central_widget)

        main_layout = QVBoxLayout(central_widget)
        main_layout.setContentsMargins(6, 6, 6, 6)
        main_layout.setSpacing(4)

        # ヘッダーレイアウト（プリセット折りたたみ・全停止・リロード）
        header_layout = QHBoxLayout()

        # プリセット表示の折りたたみ
        preset_toggle_btn = QPushButton("📋 プリセット")
        preset_toggle_btn.setCheckable(True)
        preset_toggle_btn.setChecked(True)
        preset_toggle_btn.setFixedWidth(120)
        preset_toggle_btn.setStyleSheet("background-color: #6c757d; color: white; font-weight: bold; border-radius: 4px; padding: 5px;")
        header_layout.addWidget(preset_toggle_btn)

        # タイトル
        title = QLabel("Sirius ROS2 Launch Manager")
        title_font = QFont()
        title_font.setPointSize(14)
        title_font.setBold(True)
        title.setFont(title_font)
        title.setAlignment(Qt.AlignCenter)
        header_layout.addWidget(title, 1)

        # ROS_DOMAIN_ID（全体に適用。変更は新規起動分から）
        ros_label = QLabel("ROS_DOMAIN")
        ros_label.setStyleSheet("color: #333; font-weight: bold;")
        header_layout.addWidget(ros_label)
        ros_domain_spin = QSpinBox()
        ros_domain_spin.setRange(0, 232)
        ros_domain_spin.setFixedWidth(72)
        ros_domain_spin.setToolTip("全プログラム共通の ROS_DOMAIN_ID（0-232）。変更は新しく起動する分から適用されます。")
        header_layout.addWidget(ros_domain_spin)

        # 全停止ボタン
        stop_all_btn = QPushButton("⏹ 全停止")
        stop_all_btn.setFixedWidth(110)
        stop_all_btn.setStyleSheet("background-color: #dc3545; color: white; font-weight: bold; border-radius: 4px; padding: 5px;")
        header_layout.addWidget(stop_all_btn)

        # リロードボタン
        reload_btn = QPushButton("🔄 リロード")
        reload_btn.setFixedWidth(100)
        reload_btn.setStyleSheet("background-color: #28a745; color: white; font-weight: bold; border-radius: 4px; padding: 5px;")
        header_layout.addWidget(reload_btn)

        main_layout.addLayout(header_layout)
        window.preset_toggle_btn = preset_toggle_btn
        window.stop_all_btn = stop_all_btn
        window.ros_domain_spin = ros_domain_spin

        # 情報ラベル
        info_label = QLabel("ボタンを押すとTerminatorのタブで起動します (--new-tab使用) | 緑●=起動中")
        info_label.setStyleSheet("color: gray; font-style: italic;")
        info_label.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(info_label)

        # プリセットセクション（左カラム・縦スクロール＋編集ボタン）
        preset_group = QGroupBox("プリセット")
        preset_group.setStyleSheet("QGroupBox { font-weight: bold; }")
        preset_group_layout = QVBoxLayout()
        preset_group_layout.setContentsMargins(4, 4, 4, 4)
        preset_group_layout.setSpacing(3)

        preset_edit_btn = QPushButton("✏ プリセット編集")
        preset_edit_btn.setStyleSheet(
            "background-color: #6f42c1; color: white; font-weight: bold;"
            " border-radius: 4px; padding: 5px;"
        )
        preset_group_layout.addWidget(preset_edit_btn)

        preset_scroll = QScrollArea()
        preset_scroll.setWidgetResizable(True)
        preset_scroll.setFrameShape(QFrame.NoFrame)
        preset_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        preset_content = QWidget()
        preset_layout = QVBoxLayout(preset_content)
        preset_layout.setContentsMargins(0, 0, 0, 0)
        preset_layout.setSpacing(2)
        preset_scroll.setWidget(preset_content)
        preset_group_layout.addWidget(preset_scroll, 1)

        preset_group.setLayout(preset_group_layout)
        preset_group.setMinimumWidth(260)
        window.preset_edit_btn = preset_edit_btn
        window.preset_group = preset_group

        # タブウィジェット追加
        tab_widget = QTabWidget()
        tab_layouts = {}
        if tab_names is None:
            tab_names = [
                "センサー・ハードウェア",
                "シミュレーション",
                "ユーティリティ",
                "ナビゲーション",
                "Pythonスクリプト",
                "Sirius Ear関連",
                "リアル実験",
                "オフライン・マッピング",
            ]
        for tab_name in tab_names:
            scroll = QScrollArea()
            scroll.setWidgetResizable(True)
            scroll.setFrameShape(QFrame.NoFrame)
            
            tab_content = QWidget()
            tab_layout = QVBoxLayout(tab_content)
            tab_layout.setContentsMargins(8, 8, 8, 8)
            tab_layout.setSpacing(4)
            
            scroll.setWidget(tab_content)
            tab_widget.addTab(scroll, tab_name)
            tab_layouts[tab_name] = tab_layout

        # 本体（左: プリセット 約1/3 / 右: タブ 約2/3）
        body = QSplitter(Qt.Horizontal)
        body.setChildrenCollapsible(False)
        body.addWidget(preset_group)
        body.addWidget(tab_widget)
        body.setStretchFactor(0, 1)
        body.setStretchFactor(1, 2)
        body.setSizes([420, 980])
        main_layout.addWidget(body, 1)

        # 各タブの右上に「起動中プロセス数」の赤バッジを付ける
        tab_badges = {}
        for i in range(tab_widget.count()):
            badge = QLabel("")
            badge.setAlignment(Qt.AlignCenter)
            badge.setFixedSize(20, 20)
            badge.setStyleSheet(
                "QLabel { background-color: #dc3545; color: white; font-weight: bold;"
                " border-radius: 10px; font-size: 11px; }"
            )
            badge.setVisible(False)
            tab_widget.tabBar().setTabButton(i, QTabBar.RightSide, badge)
            tab_badges[i] = badge
        window.tab_badges = tab_badges

        return preset_layout, tab_layouts, tab_widget, reload_btn
    
    @staticmethod
    def create_subgroup_header(title):
        """タブ内の小見出し（サブグループ）を作成"""
        label = QLabel(title)
        label.setObjectName("subgroupHeader")
        label.setStyleSheet(
            "QLabel#subgroupHeader {"
            " color: #0b5ed7; font-weight: bold; font-size: 13px;"
            " border-bottom: 2px solid #cfe2ff;"
            " padding: 6px 2px 3px 2px; margin-top: 8px; }"
        )
        return label

    @staticmethod
    def create_preset_button(preset_name):
        """プリセットボタンを作成（左カラム用・横幅いっぱい）"""
        preset_btn = QPushButton(f"▶ {preset_name}")
        preset_btn.setFixedHeight(26)
        preset_btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        preset_btn.setStyleSheet(
            "background-color: #007bff; color: white; font-weight: bold;"
            " font-size: 12px; text-align: left; padding: 2px 8px;"
            " border-radius: 3px;"
        )
        return preset_btn
    
    @staticmethod
    def create_group(title, description=""):
        """グループボックスを作成（説明文は既定で折りたたみ）"""
        group = QGroupBox(title)
        group_layout = QVBoxLayout()
        group.setLayout(group_layout)
        if description:
            desc_label = QLabel(description)
            desc_label.setObjectName("groupDescription")
            desc_label.setWordWrap(True)
            desc_label.setTextInteractionFlags(Qt.TextSelectableByMouse)
            desc_label.setStyleSheet(
                "QLabel#groupDescription {"
                " color: #495057; background-color: #f1f3f5;"
                " border: 1px solid #dee2e6; border-radius: 4px;"
                " padding: 7px; margin-bottom: 3px; }"
            )
            desc_label.setVisible(False)

            toggle = QPushButton("▶ ℹ 説明")
            toggle.setCheckable(True)
            toggle.setCursor(Qt.PointingHandCursor)
            toggle.setStyleSheet(
                "QPushButton { text-align: left; color: #6c757d; border: none;"
                " background: transparent; padding: 2px; font-size: 12px; }"
                "QPushButton:hover { color: #0b5ed7; }"
            )
            toggle.toggled.connect(
                lambda checked: (desc_label.setVisible(checked),
                                 toggle.setText("▼ ℹ 説明" if checked else "▶ ℹ 説明"))
            )
            group_layout.addWidget(toggle)
            group_layout.addWidget(desc_label)
        return group, group_layout
