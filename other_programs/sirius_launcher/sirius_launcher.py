#!/usr/bin/env python3
"""
Sirius ROS2 Launch Manager
ROSノードやlaunchファイルをGUIボタンから起動するランチャーアプリケーション
"""

import sys
import os
import subprocess
import signal
import re
from pathlib import Path
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QScrollArea, QFrame, QMessageBox, QGroupBox
)
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QFont


def parse_bash_aliases(alias_file_path):
    """bash_alias2ファイルからエイリアスを解析（複数行対応）"""
    groups = {}
    current_group = "その他"
    current_description = ""
    
    try:
        with open(alias_file_path, 'r', encoding='utf-8') as f:
            content = f.read()
        
        lines = content.split('\n')
        i = 0
        while i < len(lines):
            line = lines[i].strip()
            
            if not line:
                current_description = ""
                i += 1
                continue
            
            if line.startswith('# GROUP:'):
                current_group = line.replace('# GROUP:', '').strip()
                if current_group not in groups:
                    groups[current_group] = []
                current_description = ""
                i += 1
                continue
            
            if line.startswith('#') and not line.startswith('# GROUP:'):
                current_description = line.lstrip('#').strip()
                i += 1
                continue
            
            if line.startswith('alias '):
                # 複数行のエイリアスを結合
                full_line = line
                while full_line.endswith('\\') and i + 1 < len(lines):
                    i += 1
                    full_line = full_line[:-1] + ' ' + lines[i].strip()
                
                # エイリアスをパース
                match = re.match(r"alias\s+([^=]+)='(.+)'", full_line, re.DOTALL)
                if match:
                    alias_name = match.group(1).strip()
                    command = match.group(2).strip()
                    
                    if alias_name in ['install_packages']:
                        i += 1
                        continue
                    
                    # src && を実際のコマンドに展開
                    if command.startswith('src && '):
                        command = command.replace('src && ', 
                            'cd ~/sirius_jazzy_ws && source ~/sirius_jazzy_ws/install/setup.bash && ')
                    
                    description = current_description if current_description else alias_name
                    
                    if current_group not in groups:
                        groups[current_group] = []
                    groups[current_group].append((alias_name, command, description))
                    
                    current_description = ""
            
            i += 1
    
    except Exception as e:
        print(f"エイリアスファイルの読み込みエラー: {e}")
    
    return groups


class LaunchButton(QWidget):
    """起動ボタンとステータス表示を含むウィジェット"""
    
    def __init__(self, name, command, description="", launcher=None):
        super().__init__()
        self.name = name
        self.command = command
        self.description = description
        self.launcher = launcher
        self.window_id = None
        self.window_title = None
        self.just_launched = False  # 起動直後かどうかのフラグ
        self.setup_ui()
    
    def setup_ui(self):
        """UIのセットアップ"""
        layout = QHBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)
        
        self.launch_btn = QPushButton(self.name)
        self.launch_btn.setMinimumWidth(150)
        self.launch_btn.clicked.connect(self.launch)
        layout.addWidget(self.launch_btn)
        
        self.status_label = QLabel("停止中")
        self.status_label.setMinimumWidth(80)
        self.update_status_style(False)
        layout.addWidget(self.status_label)
        
        desc_label = QLabel(self.description)
        desc_label.setStyleSheet("color: gray;")
        layout.addWidget(desc_label, 1)
        
        self.focus_btn = QPushButton("ウィンドウ表示")
        self.focus_btn.setEnabled(False)
        self.focus_btn.clicked.connect(self.focus_window)
        layout.addWidget(self.focus_btn)
        
        self.stop_btn = QPushButton("停止")
        self.stop_btn.setEnabled(False)
        self.stop_btn.clicked.connect(self.stop)
        layout.addWidget(self.stop_btn)
    
    def update_status_style(self, is_running):
        """ステータス表示の更新"""
        if is_running:
            self.status_label.setText("起動中")
            self.status_label.setStyleSheet("color: green; font-weight: bold;")
        else:
            self.status_label.setText("停止中")
            self.status_label.setStyleSheet("color: gray;")
    
    def launch(self):
        """コマンドを新しいターミナルで起動"""
        if self.is_running():
            self.focus_window()
            return
        
        try:
            # ユニークなウィンドウタイトルを作成
            import time
            unique_title = f"SIRIUS_{self.name}_{int(time.time()*1000)}"
            self.window_title = unique_title
            self.just_launched = True  # 起動直後フラグ
            
            # コマンドをシェルスクリプトとして一時ファイルに保存
            import tempfile
            with tempfile.NamedTemporaryFile(mode='w', suffix='.sh', delete=False) as f:
                f.write('#!/bin/bash\n')
                f.write(f'# Window title: {unique_title}\n')
                f.write(f'echo -ne "\\033]0;{unique_title}\\007"\n')  # タイトルを設定
                f.write(f'{self.command}\n')
                f.write('exec bash\n')
                script_path = f.name
            
            os.chmod(script_path, 0o755)
            
            # シェル経由で起動（環境変数を確実に継承）
            shell_cmd = f'terminator --title="{unique_title}" -x bash "{script_path}" &'
            
            print(f"実行コマンド: {shell_cmd}")
            
            process = subprocess.Popen(
                shell_cmd,
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                env=os.environ.copy()
            )
            
            # スクリプトファイルは後で削除
            QTimer.singleShot(8000, lambda: self.cleanup_script(script_path))
            
            # ウィンドウIDを取得するまで待つ
            QTimer.singleShot(1000, self.get_window_id)
            QTimer.singleShot(2000, self.get_window_id)
            QTimer.singleShot(3000, self.get_window_id)
            QTimer.singleShot(4500, self.clear_launch_flag)
            
            self.launch_btn.setEnabled(False)
            self.stop_btn.setEnabled(True)
            self.focus_btn.setEnabled(True)
            self.update_status_style(True)
            
            print(f"起動: {self.name} (ウィンドウタイトル: {unique_title})")
            
        except Exception as e:
            QMessageBox.critical(self, "エラー", f"起動に失敗しました: {str(e)}")
            print(f"起動エラー: {e}")
    
    def cleanup_script(self, script_path):
        """一時スクリプトファイルを削除"""
        try:
            if os.path.exists(script_path):
                os.remove(script_path)
                print(f"スクリプト削除: {script_path}")
        except Exception as e:
            print(f"スクリプト削除エラー: {e}")
    
    def is_running(self):
        """プロセスが実行中かチェック"""
        if not self.window_title:
            return False
        
        try:
            # ウィンドウが存在するかチェック
            result = subprocess.run(
                ["wmctrl", "-l"],
                capture_output=True,
                text=True,
                timeout=2
            )
            is_found = self.window_title in result.stdout
            if is_found and not self.window_id:
                # ウィンドウIDをまだ取得していない場合は取得
                self.get_window_id()
            return is_found
        except Exception as e:
            print(f"is_running エラー: {e}")
            return False
    
    def get_window_id(self):
        """起動したターミナルのウィンドウIDを取得"""
        if not self.window_title:
            return
        
        try:
            result = subprocess.run(
                ["wmctrl", "-l"],
                capture_output=True,
                text=True,
                timeout=2
            )
            lines = result.stdout.split('\n')
            print(f"=== ウィンドウ一覧 (検索: {self.window_title}) ===")
            for line in lines:
                if line.strip():
                    print(f"  {line}")
                if self.window_title in line:
                    parts = line.split()
                    if parts:
                        self.window_id = parts[0]
                        print(f"✓ ウィンドウID取得: {self.name} -> {self.window_id}")
                        return
            print(f"✗ ウィンドウが見つかりません: {self.window_title}")
        except Exception as e:
            print(f"get_window_id エラー: {e}")
    
    def focus_window(self):
        """起動したターミナルウィンドウにフォーカス"""
        # ウィンドウIDを再取得
        self.get_window_id()
        
        print(f"フォーカス試行: {self.name}, ID: {self.window_id}")
        
        if self.window_id:
            try:
                result = subprocess.run(
                    ["wmctrl", "-i", "-a", self.window_id],
                    capture_output=True,
                    text=True,
                    timeout=2
                )
                if result.returncode != 0:
                    print(f"wmctrl エラー: {result.stderr}")
                    QMessageBox.warning(self, "警告", "ウィンドウのフォーカスに失敗しました")
            except Exception as e:
                print(f"focus_window エラー: {e}")
                QMessageBox.warning(self, "警告", f"ウィンドウのフォーカスに失敗しました: {e}")
        else:
            QMessageBox.information(self, "情報", "ウィンドウが見つかりません")
    
    def stop(self):
        """プロセスを停止"""
        # ウィンドウIDを再取得
        self.get_window_id()
        
        print(f"停止試行: {self.name}, ID: {self.window_id}")
        
        if self.window_id:
            try:
                # ウィンドウを閉じる
                result = subprocess.run(
                    ["wmctrl", "-i", "-c", self.window_id],
                    capture_output=True,
                    text=True,
                    timeout=2
                )
                if result.returncode != 0:
                    print(f"wmctrl エラー: {result.stderr}")
                QTimer.singleShot(500, self.reset_state)
            except Exception as e:
                print(f"stop エラー: {e}")
                QMessageBox.warning(self, "警告", f"停止に失敗しました: {str(e)}")
                self.reset_state()
        else:
            QMessageBox.information(self, "情報", "ウィンドウが見つかりません\nすでに閉じられている可能性があります")
            self.reset_state()
    
    def reset_state(self):
        """ボタンの状態をリセット"""
        self.window_id = None
        self.window_title = None
        self.just_launched = False
        self.launch_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
        self.focus_btn.setEnabled(False)
        self.update_status_style(False)
    
    def check_status(self):
        """プロセスの状態をチェック"""
        if self.window_title and not self.just_launched:
            # 起動直後でない場合のみチェック
            if not self.is_running():
                # ウィンドウが閉じられた場合、状態をリセット
                print(f"ウィンドウ閉じられた: {self.name}")
                self.reset_state()
    
    def clear_launch_flag(self):
        """起動直後フラグをクリア"""
        self.just_launched = False
        print(f"起動フラグクリア: {self.name}")
    
    def check_launch_error(self, process):
        """起動エラーをチェック"""
        if process.poll() is not None:
            # プロセスが終了している場合、エラーを確認
            stdout, stderr = process.communicate()
            if stderr:
                error_msg = stderr.decode('utf-8', errors='ignore')
                print(f"Terminatorエラー ({self.name}): {error_msg}")
                QMessageBox.critical(self, "Terminatorエラー", f"ターミナルの起動に失敗しました:\n{error_msg}")
                self.reset_state()


class SiriusLauncher(QMainWindow):
    """Sirius ROS2 Launch Manager メインウィンドウ"""
    
    def __init__(self):
        super().__init__()
        self.buttons = []
        self.first_terminal = True
        self.setup_ui()
        self.load_aliases()
        
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.check_all_status)
        self.timer.start(2000)
    
    def setup_ui(self):
        """UIのセットアップ"""
        self.setWindowTitle("Sirius ROS2 Launch Manager")
        self.setMinimumSize(900, 600)
        
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        main_layout = QVBoxLayout(central_widget)
        main_layout.setContentsMargins(10, 10, 10, 10)
        
        title = QLabel("Sirius ROS2 Launch Manager")
        title_font = QFont()
        title_font.setPointSize(16)
        title_font.setBold(True)
        title.setFont(title_font)
        title.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(title)
        
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        
        scroll_content = QWidget()
        self.buttons_layout = QVBoxLayout(scroll_content)
        self.buttons_layout.setSpacing(5)
        
        scroll.setWidget(scroll_content)
        main_layout.addWidget(scroll)
        
        stop_all_btn = QPushButton("全て停止")
        stop_all_btn.clicked.connect(self.stop_all)
        stop_all_btn.setStyleSheet("background-color: #ff4444; color: white; font-weight: bold;")
        stop_all_btn.setMinimumHeight(40)
        main_layout.addWidget(stop_all_btn)
    
    def add_group(self, title):
        """グループボックスを追加"""
        group = QGroupBox(title)
        group_layout = QVBoxLayout()
        group.setLayout(group_layout)
        self.buttons_layout.addWidget(group)
        return group_layout
    
    def load_aliases(self):
        """エイリアスファイルを読み込んでボタンを作成"""
        alias_file = Path.home() / "sirius_jazzy_ws" / "bash" / ".bash_alias2"
        
        if not alias_file.exists():
            QMessageBox.warning(self, "警告", f"エイリアスファイルが見つかりません: {alias_file}")
            return
        
        groups = parse_bash_aliases(str(alias_file))
        
        if not groups:
            QMessageBox.warning(self, "警告", "エイリアスが見つかりませんでした。")
            return
        
        for group_name, aliases in groups.items():
            if aliases:
                group_layout = self.add_group(group_name)
                for alias_name, command, description in aliases:
                    self.add_button(group_layout, alias_name, command, description)
        
        self.buttons_layout.addStretch()
    
    def add_button(self, layout, name, command, description):
        """ボタンを追加"""
        button = LaunchButton(name, command, description, launcher=self)
        layout.addWidget(button)
        self.buttons.append(button)
    
    def check_all_status(self):
        """全ボタンのステータスをチェック"""
        for button in self.buttons:
            button.check_status()
    
    def stop_all(self):
        """全てのプロセスを停止"""
        reply = QMessageBox.question(
            self, "確認",
            "全てのプロセスを停止しますか？",
            QMessageBox.Yes | QMessageBox.No
        )
        
        if reply == QMessageBox.Yes:
            for button in self.buttons:
                button.stop()


def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    
    window = SiriusLauncher()
    window.show()
    
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
