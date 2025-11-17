#!/usr/bin/env python3
"""
Sirius ROS2 Launch Manager (New Tab Version)
ROSノードやlaunchファイルをGUIボタンから起動するランチャーアプリケーション
Terminatorの--new-tabオプションを使用したシンプル版
起動状態追跡と停止ボタン機能付き
"""

import sys
import os
import subprocess
import re
import signal
import psutil
from pathlib import Path
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QScrollArea, QFrame, QMessageBox, QGroupBox
)
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QFont, QColor


def parse_bash_aliases(alias_file_path):
    """bash_alias2ファイルからエイリアスとプリセットを解析（複数行対応）"""
    groups = {}
    presets = []  # プリセットリスト
    current_group = "その他"
    current_description = ""
    current_preset_name = None
    
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
            
            # プリセット名の検出
            if line.startswith('# PRESET:'):
                current_preset_name = line.replace('# PRESET:', '').strip()
                i += 1
                continue
            
            # プリセットアイテムの検出
            if line.startswith('# PRESET_ITEMS:') and current_preset_name:
                items_str = line.replace('# PRESET_ITEMS:', '').strip()
                items = [item.strip() for item in items_str.split(',')]
                presets.append((current_preset_name, items))
                current_preset_name = None
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
    
    return groups, presets


class LaunchButton(QWidget):
    """起動ボタンを含むウィジェット"""
    
    def __init__(self, name, command, description=""):
        super().__init__()
        self.name = name
        self.command = command
        self.description = description
        self.process = None
        self.pid_file = f"/tmp/sirius_launcher_{name}.pid"
        self.tab_title = name  # Sirius_を削除
        self.setup_ui()
        
        # 起動時に古いPIDファイルをチェック
        self.load_pid()
        
        # 定期的にプロセス状態をチェック
        self.timer = QTimer()
        self.timer.timeout.connect(self.check_process_status)
        self.timer.start(1000)  # 1秒ごとにチェック
    
    def setup_ui(self):
        """UIのセットアップ"""
        layout = QHBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)
        
        # 起動ボタン
        self.launch_btn = QPushButton(f"▶ {self.name}")
        self.launch_btn.setMinimumWidth(200)
        self.launch_btn.clicked.connect(self.launch)
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
        self.stop_btn.clicked.connect(self.stop)
        self.stop_btn.setEnabled(False)
        layout.addWidget(self.stop_btn)
    
    def launch(self):
        """コマンドを新しいターミナルタブで起動"""
        try:
            # 一時スクリプトファイルを作成
            import tempfile
            script_fd, script_path = tempfile.mkstemp(suffix='.sh', prefix='sirius_launcher_')
            
            with os.fdopen(script_fd, 'w') as f:
                f.write('#!/bin/bash\n')
                f.write(f'echo $BASHPID > {self.pid_file}\n')
                # プロンプトにタブ名を表示（PS1に設定）
                f.write(f'export PS1="[{self.tab_title}] $ "\n')
                # ウィンドウタイトルを設定
                f.write(f'echo -ne "\\033]0;{self.tab_title}\\007"\n')
                f.write(f'{self.command}\n')
            
            os.chmod(script_path, 0o755)
            
            # Terminatorで新しいタブを開く
            shell_cmd = f'terminator --new-tab -e "bash --rcfile {script_path}"'
            
            self.process = subprocess.Popen(shell_cmd, shell=True, env=os.environ.copy())
            self.script_path = script_path
            
            # PIDファイルが作成されるまで待つ
            QTimer.singleShot(1500, self.load_pid)
            
            print(f"起動: {self.name} (Terminatorタブ: {self.tab_title})")
            
        except Exception as e:
            QMessageBox.critical(self, "エラー", f"起動に失敗しました: {str(e)}")
            print(f"起動エラー: {e}")
    
    def load_pid(self, retry_count=0):
        """PIDファイルからプロセスIDを読み込む"""
        try:
            if os.path.exists(self.pid_file):
                with open(self.pid_file, 'r') as f:
                    pid_str = f.read().strip()
                    if not pid_str:
                        # ファイルが空の場合、後で再試行
                        if retry_count < 10:
                            QTimer.singleShot(500, lambda: self.load_pid(retry_count + 1))
                        return
                    
                    pid = int(pid_str)
                    if psutil.pid_exists(pid):
                        # プロセスが実際に存在するか確認
                        try:
                            proc = psutil.Process(pid)
                            # bashプロセスなので、その子プロセスを探す
                            children = proc.children(recursive=True)  # recursive=Trueに変更
                            if children:
                                # すべての子孫プロセスを含む
                                self.pid_file_content = pid
                                print(f"プロセス検出: {self.name} (Bash PID: {pid}, 全子プロセス: {len(children)}個)")
                                self.update_status()
                            else:
                                # 子プロセスがまだない場合は後でもう一度試す（最大10回）
                                if retry_count < 10:
                                    QTimer.singleShot(500, lambda: self.load_pid(retry_count + 1))
                                else:
                                    # 子プロセスがなくてもbashプロセスは記録
                                    self.pid_file_content = pid
                                    print(f"プロセス検出(子なし): {self.name} (Bash PID: {pid})")
                                    self.update_status()
                        except psutil.NoSuchProcess:
                            # プロセスが既に終了している
                            if os.path.exists(self.pid_file):
                                os.remove(self.pid_file)
                    else:
                        # PIDが存在しない場合、ファイルを削除
                        if os.path.exists(self.pid_file):
                            os.remove(self.pid_file)
            else:
                # PIDファイルがまだ作成されていない場合、後で再試行
                if retry_count < 10:
                    QTimer.singleShot(500, lambda: self.load_pid(retry_count + 1))
        except Exception as e:
            print(f"PID読み込みエラー ({self.name}): {e}")
    
    def get_process_tree_pids(self, pid):
        """プロセスとその子孫のPIDリストを取得"""
        pids = [pid]
        try:
            proc = psutil.Process(pid)
            children = proc.children(recursive=True)
            for child in children:
                pids.append(child.pid)
        except:
            pass
        return pids
    
    def stop(self):
        """プロセスを停止"""
        try:
            if hasattr(self, 'pid_file_content') and self.pid_file_content:
                # プロセスツリー全体のPIDを取得
                pids = self.get_process_tree_pids(self.pid_file_content)
                
                if len(pids) > 1:
                    print(f"停止対象PID: Bash={self.pid_file_content}, 子孫={len(pids)-1}個")
                else:
                    print(f"停止対象PID: Bash={self.pid_file_content}, 子孫なし")
                
                # Terminatorのタブを見つける
                terminator_pid = None
                for pid in pids:
                    try:
                        proc = psutil.Process(pid)
                        if proc.name() == 'terminator':
                            terminator_pid = pid
                            break
                    except:
                        pass
                
                # 子プロセスから順に終了（逆順）、terminatorは除外
                for pid in reversed(pids):
                    if pid == terminator_pid:
                        continue  # terminatorプロセスはスキップ
                    try:
                        if psutil.pid_exists(pid):
                            proc = psutil.Process(pid)
                            proc.terminate()
                            print(f"  terminate: PID {pid} ({proc.name()})")
                    except (psutil.NoSuchProcess, psutil.AccessDenied) as e:
                        print(f"  スキップ: PID {pid} - {e}")
                
                # 2秒待っても終了しない場合は強制終了
                QTimer.singleShot(2000, lambda: self.force_kill_tree(pids, terminator_pid))
                
                # Terminatorのタブを閉じる
                QTimer.singleShot(2500, lambda: self.close_terminator_tab(terminator_pid))
                
                print(f"停止: {self.name} (Bash PID: {self.pid_file_content})")
                
                # PIDファイルを削除
                if os.path.exists(self.pid_file):
                    os.remove(self.pid_file)
                
                # 一時スクリプトファイルを削除
                if hasattr(self, 'script_path') and os.path.exists(self.script_path):
                    try:
                        os.remove(self.script_path)
                    except:
                        pass
                
                self.pid_file_content = None
                self.update_status()
            else:
                print(f"停止対象なし: {self.name}")
            
        except Exception as e:
            QMessageBox.critical(self, "エラー", f"停止に失敗しました: {str(e)}")
            print(f"停止エラー: {e}")
    
    def force_kill_tree(self, pids, terminator_pid=None):
        """プロセスツリーを強制終了"""
        killed_count = 0
        for pid in reversed(pids):
            if pid == terminator_pid:
                continue  # terminatorプロセスはスキップ
            try:
                if psutil.pid_exists(pid):
                    proc = psutil.Process(pid)
                    proc.kill()
                    killed_count += 1
                    print(f"  強制終了: PID {pid} ({proc.name()})")
            except:
                pass
        if killed_count > 0:
            print(f"強制終了完了: {self.name} ({killed_count}個のプロセス)")
    
    def close_terminator_tab(self, terminator_pid):
        """Terminatorのタブを閉じる"""
        try:
            if terminator_pid and psutil.pid_exists(terminator_pid):
                # Terminatorウィンドウにフォーカスして Ctrl+Shift+W でタブを閉じる
                output = subprocess.check_output(['wmctrl', '-lp'], text=True)
                for line in output.splitlines():
                    parts = line.split()
                    if len(parts) >= 4:
                        window_id = parts[0]
                        window_pid = int(parts[2])
                        if window_pid == terminator_pid:
                            subprocess.run(['wmctrl', '-i', '-a', window_id])
                            subprocess.run(['xdotool', 'key', 'ctrl+shift+w'])
                            print(f"Terminatorタブを閉じました: {self.name}")
                            return
        except Exception as e:
            print(f"タブを閉じるエラー: {e}")
    
    def check_process_status(self):
        """プロセスの状態を定期的にチェック"""
        if hasattr(self, 'pid_file_content') and self.pid_file_content:
            if not psutil.pid_exists(self.pid_file_content):
                # プロセスが終了している
                print(f"プロセス終了検出: {self.name}")
                self.pid_file_content = None
                if os.path.exists(self.pid_file):
                    os.remove(self.pid_file)
                self.update_status()
    
    def update_status(self):
        """ステータス表示を更新"""
        is_running = hasattr(self, 'pid_file_content') and self.pid_file_content is not None
        
        # ボタンの状態を更新
        self.launch_btn.setEnabled(not is_running)
        self.stop_btn.setEnabled(is_running)
        
        # ステータスインジケータを更新
        if is_running:
            self.status_label.setStyleSheet("color: #28a745; font-size: 16px;")  # 緑
            self.status_label.setText("●")
        else:
            self.status_label.setStyleSheet("color: gray; font-size: 16px;")
            self.status_label.setText("●")


class SiriusLauncher(QMainWindow):
    """Sirius ROS2 Launch Manager メインウィンドウ"""
    
    def __init__(self):
        super().__init__()
        self.buttons = []
        self.button_map = {}  # 名前からボタンへのマッピング
        self.presets = []
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
        
        info_label = QLabel("ボタンを押すとTerminatorのタブで起動します (--new-tab使用) | 緑●=起動中")
        info_label.setStyleSheet("color: gray; font-style: italic;")
        info_label.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(info_label)
        
        # プリセットセクション
        self.preset_group = QGroupBox("プリセット")
        self.preset_group.setStyleSheet("QGroupBox { font-weight: bold; }")
        self.preset_layout = QHBoxLayout()
        self.preset_group.setLayout(self.preset_layout)
        main_layout.addWidget(self.preset_group)
        
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
        
        groups, presets = parse_bash_aliases(str(alias_file))
        self.presets = presets
        
        if not groups:
            QMessageBox.warning(self, "警告", "エイリアスが見つかりませんでした。")
            return
        
        # プリセットボタンを作成
        for preset_name, items in presets:
            self.add_preset_button(preset_name, items)
        
        # 通常のボタンを作成
        for group_name, aliases in groups.items():
            if aliases:
                group_layout = self.add_group(group_name)
                for alias_name, command, description in aliases:
                    self.add_button(group_layout, alias_name, command, description)
        
        self.buttons_layout.addStretch()
    
    def add_preset_button(self, preset_name, items):
        """プリセットボタンを追加"""
        preset_btn = QPushButton(f"▶ {preset_name}")
        preset_btn.setMinimumHeight(40)
        preset_btn.setStyleSheet("background-color: #007bff; color: white; font-weight: bold;")
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
                if not (hasattr(button, 'pid_file_content') and button.pid_file_content):
                    button.launch()
                    time.sleep(0.5)  # 各起動の間に少し待つ
                else:
                    print(f"  スキップ: {item} (既に起動中)")
            else:
                print(f"  エラー: {item} が見つかりません")
    
    def add_button(self, layout, name, command, description):
        """ボタンを追加"""
        button = LaunchButton(name, command, description)
        layout.addWidget(button)
        self.buttons.append(button)
        self.button_map[name] = button  # 名前で検索できるように保存


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
