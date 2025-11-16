#!/usr/bin/env python3
"""
Sirius ROS2 Launch Manager (Simple Version)
ROSノードやlaunchファイルをGUIボタンから起動するランチャーアプリケーション
起動のみ実装、プロセス追跡なし
"""

import sys
import os
import subprocess
import re
from pathlib import Path
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QScrollArea, QFrame, QMessageBox, QGroupBox
)
from PySide6.QtCore import Qt
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
    """起動ボタンを含むウィジェット"""
    
    def __init__(self, name, command, description="", launcher=None):
        super().__init__()
        self.name = name
        self.command = command
        self.description = description
        self.launcher = launcher
        self.setup_ui()
    
    def setup_ui(self):
        """UIのセットアップ"""
        layout = QHBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)
        
        self.launch_btn = QPushButton(self.name)
        self.launch_btn.setMinimumWidth(200)
        self.launch_btn.clicked.connect(self.launch)
        layout.addWidget(self.launch_btn)
        
        desc_label = QLabel(self.description)
        desc_label.setStyleSheet("color: gray;")
        layout.addWidget(desc_label, 1)
    
    def launch(self):
        """コマンドを新しいターミナルタブで起動"""
        try:
            # 一時スクリプトファイルを作成
            import tempfile
            import time
            
            with tempfile.NamedTemporaryFile(mode='w', suffix='.sh', delete=False) as f:
                f.write('#!/bin/bash\n')
                f.write(f'# {self.name}\n')
                f.write(f'{self.command}\n')
                f.write('exec bash\n')
                script_path = f.name
            
            os.chmod(script_path, 0o755)
            
            # タブで起動（最初のウィンドウか、既存のウィンドウに追加）
            if self.launcher and self.launcher.first_launch:
                # 最初の起動は新しいウィンドウ
                unique_title = f"SiriusLauncher_{int(time.time()*1000)}"
                shell_cmd = f'terminator --title="{unique_title}" -x bash "{script_path}" &'
                self.launcher.first_launch = False
                self.launcher.first_window_title = unique_title
                print(f"起動: {self.name} (新しいウィンドウ: {unique_title})")
            else:
                # 2回目以降は1個目のウィンドウを見つけてキーボードショートカット(Ctrl+Shift+T)でタブ追加
                window_title = self.launcher.first_window_title
                shell_cmd = f'''
WINDOW_ID=$(wmctrl -l | grep "{window_title}" | head -n1 | awk '{{print $1}}')
if [ -n "$WINDOW_ID" ]; then
    wmctrl -i -a "$WINDOW_ID"
    sleep 0.5
    xdotool key ctrl+shift+t
    sleep 0.5
    xdotool type "bash {script_path}"
    xdotool key Return
else
    terminator --title="{self.name}" -x bash "{script_path}" &
fi
''' 
                print(f"起動: {self.name} (タブ追加: {window_title})")
            
            subprocess.Popen(
                shell_cmd,
                shell=True,
                env=os.environ.copy()
            )
            
            # 10秒後にスクリプトファイルを削除
            from PySide6.QtCore import QTimer
            QTimer.singleShot(15000, lambda: self.cleanup_script(script_path))
            
        except Exception as e:
            QMessageBox.critical(self, "エラー", f"起動に失敗しました: {str(e)}")
            print(f"起動エラー: {e}")
    
    def cleanup_script(self, script_path):
        """一時スクリプトファイルを削除"""
        try:
            if os.path.exists(script_path):
                os.remove(script_path)
        except Exception as e:
            print(f"スクリプト削除エラー: {e}")


class SiriusLauncher(QMainWindow):
    """Sirius ROS2 Launch Manager メインウィンドウ"""
    
    def __init__(self):
        super().__init__()
        self.buttons = []
        self.first_launch = True
        self.first_window_title = None
        self.setup_ui()
        self.load_aliases()
    
    def setup_ui(self):
        """UIのセットアップ"""
        self.setWindowTitle("Sirius ROS2 Launch Manager")
        self.setMinimumSize(800, 600)
        
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
        
        info_label = QLabel("ボタンを押すとTerminatorのタブで起動します")
        info_label.setStyleSheet("color: gray; font-style: italic;")
        info_label.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(info_label)
        
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        
        scroll_content = QWidget()
        self.buttons_layout = QVBoxLayout(scroll_content)
        self.buttons_layout.setSpacing(5)
        
        scroll.setWidget(scroll_content)
        main_layout.addWidget(scroll)
    
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


def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    
    window = SiriusLauncher()
    window.show()
    
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
