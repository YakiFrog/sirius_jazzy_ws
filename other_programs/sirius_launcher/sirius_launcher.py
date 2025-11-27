#!/usr/bin/env python3
"""
Sirius ROS2 Launch Manager (New Tab Version)
ROSノードやlaunchファイルをGUIボタンから起動するランチャーアプリケーション
Terminatorの--new-tabオプションを使用したシンプル版
起動状態追跡と停止ボタン機能付き
"""

import sys
import signal
from pathlib import Path
from PySide6.QtWidgets import QApplication, QMainWindow, QMessageBox
from PySide6.QtCore import QTimer, QEvent

from alias_parser import parse_bash_aliases
from ui_components import LaunchButtonUI, MainWindowUI
from process_manager import ProcessManager


class LaunchButton(LaunchButtonUI):
    """起動ボタンを含むウィジェット（ロジック統合版）"""
    
    def __init__(self, name, command, description="", tab_widget=None, tab_index=None):
        super().__init__(name, description)
        # Tab control info
        self.tab_widget = tab_widget
        self.tab_index = tab_index
        self.command = command
        self.process_manager = ProcessManager(name, command)
        
        # ボタンのイベント接続
        self.launch_btn.clicked.connect(self.launch)
        self.stop_btn.clicked.connect(self.stop)
        # クリックでタブ選択（実行中のみ）
        self.launch_btn.clicked.connect(self.select_tab_if_running)
        self.stop_btn.clicked.connect(self.select_tab_if_running)
        
        # 起動時に古いPIDファイルをチェック
        self.load_pid()
        
        # 定期的にプロセス状態をチェック
        self.timer = QTimer()
        self.timer.timeout.connect(self.check_process_status)
        self.timer.start(1000)
        # イベントフィルタをインストールして、ウィジェット全体のクリックを捕まえる
        self.installEventFilter(self)
        # 子ウィジェットにもフィルタをインストール
        try:
            self.launch_btn.installEventFilter(self)
            self.stop_btn.installEventFilter(self)
            self.status_label.installEventFilter(self)
            # desc_labelはui_componentsでself.desc_labelとして公開済み
            if hasattr(self, 'desc_label'):
                self.desc_label.installEventFilter(self)
        except Exception:
            pass
    
    def launch(self):
        """コマンドを新しいターミナルタブで起動"""
        if self.process_manager.launch():
            QTimer.singleShot(1500, self.load_pid)
            print(f"起動: {self.name}")
        else:
            QMessageBox.critical(self, "エラー", f"起動に失敗しました: {self.name}")
    
    def load_pid(self, retry_count=0):
        """PIDファイルからプロセスIDを読み込む"""
        result = self.process_manager.load_pid(retry_count)
        if result == "retry":
            QTimer.singleShot(500, lambda: self.load_pid(retry_count + 1))
        elif result:
            self.update_status(True)
    
    def stop(self):
        """プロセスを停止"""
        result = self.process_manager.stop()
        if result:
            pids = result['pids']
            terminator_pid = result['terminator_pid']
            
            # 2秒待っても終了しない場合は強制終了
            QTimer.singleShot(2000, lambda: self.process_manager.force_kill_tree(pids, terminator_pid))
            
            # Terminatorのタブを閉じる
            QTimer.singleShot(2500, lambda: self.process_manager.close_terminator_tab(terminator_pid))
            
            self.update_status(False)
        else:
            QMessageBox.critical(self, "エラー", "停止に失敗しました")
    
    def check_process_status(self):
        """プロセスの状態を定期的にチェック"""
        is_running = self.process_manager.is_running()
        current_state = self.launch_btn.isEnabled() == False
        if is_running != current_state:
            self.update_status(is_running)

    def select_tab(self):
        """この項目があるタブを選択する"""
        if self.tab_widget is not None and self.tab_index is not None and self.tab_index >= 0:
            try:
                self.tab_widget.setCurrentIndex(self.tab_index)
            except Exception:
                pass

    def select_tab_if_running(self):
        """実行中のときだけタブを選択するユーティリティ"""
        if self.process_manager.is_running():
            self.select_tab()

    def eventFilter(self, watched, event):
        # マウスクリックイベントを捕まえてタブ選択を行う
        if event.type() == QEvent.MouseButtonPress:
            # クリックされたとき、実行中ならタブ選択
            if self.process_manager.is_running():
                self.select_tab()
        return super().eventFilter(watched, event)


class SiriusLauncher(QMainWindow):
    """Sirius ROS2 Launch Manager メインウィンドウ"""
    
    def __init__(self):
        super().__init__()
        self.buttons = []
        self.button_map = {}
        self.presets = []
        self.setup_ui()
        self.load_aliases()
    
    def setup_ui(self):
        """UIのセットアップ"""
        self.preset_layout, self.tab_layouts, self.tab_widget = MainWindowUI.setup_ui(self)

    def add_group(self, title, tab_name=None):
        """グループボックスを追加（タブ対応）"""
        group, group_layout = MainWindowUI.create_group(title)
        # タブ名指定がなければ最初のタブに追加
        if tab_name is None:
            tab_name = list(self.tab_layouts.keys())[0]
        self.tab_layouts[tab_name].addWidget(group)
        return group_layout, group
    
    # NOTE: previous single-tab add_group kept for reference is removed
    
    def load_aliases(self):
        """エイリアスファイルを読み込んでボタンを作成"""
        alias_file = Path.home() / "sirius_jazzy_ws" / "bash" / "bash_alias2.sh"

        if not alias_file.exists():
            QMessageBox.warning(self, "警告", f"エイリアスファイルが見つかりません: {alias_file}")
            return

        groups, presets = parse_bash_aliases(str(alias_file))
        self.presets = presets

        if not groups:
            QMessageBox.warning(self, "警告", "エイリアスが見つかりませんでした。")
            return

        # プリセットボタンを作成
        for preset_name, items in presets:
            self.add_preset_button(preset_name, items)

        # タブ名リスト（ui_components.pyのデフォルトと合わせる）
        tab_names = ["センサー・ハードウェア", "シミュレーション", "ユーティリティ", "ナビゲーション", "Pythonスクリプト", "Sirius Ear関連"]

        # 通常のボタンを作成（タブごとにグループ追加）
        for i, (group_name, aliases) in enumerate(groups.items()):
            if aliases:
                tab_name = tab_names[i] if i < len(tab_names) else tab_names[0]
                group_layout, group_widget = self.add_group(group_name, tab_name)
                for alias_name, command, description in aliases:
                    self.add_button(group_layout, alias_name, command, description, group_widget)

        # 各タブのレイアウトにストレッチ追加
        for layout in self.tab_layouts.values():
            layout.addStretch()
    
    def add_preset_button(self, preset_name, items):
        """プリセットボタンを追加"""
        preset_btn = MainWindowUI.create_preset_button(preset_name)
        preset_btn.clicked.connect(lambda: self.launch_preset(preset_name, items))
        self.preset_layout.addWidget(preset_btn)
    
    def launch_preset(self, preset_name, items):
        """プリセットの複数コマンドを同時起動"""
        print(f"プリセット起動: {preset_name}")
        import time
        for item in items:
            if item in self.button_map:
                button = self.button_map[item]
                # 起動中でない場合のみ起動
                if not button.process_manager.is_running():
                    button.launch()
                    time.sleep(0.5)
                else:
                    print(f"  スキップ: {item} (既に起動中)")
            else:
                print(f"  エラー: {item} が見つかりません")
    
    def add_button(self, layout, name, command, description, group_widget=None):
        """ボタンを追加"""
        tab_index = None
        if group_widget is not None and self.tab_widget is not None:
            tab_index = self.tab_widget.indexOf(group_widget.parentWidget())
        button = LaunchButton(name, command, description, tab_widget=self.tab_widget, tab_index=tab_index)
        layout.addWidget(button)
        self.buttons.append(button)
        self.button_map[name] = button


def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    
    window = SiriusLauncher()
    window.show()
    
    # Ctrl+Cでの終了を処理
    import signal
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
