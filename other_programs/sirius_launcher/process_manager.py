"""
Sirius ROS2 Launch Manager - Process Management
プロセス管理とコマンド実行の実装
"""

import os
import subprocess
import tempfile
import psutil
from PySide6.QtCore import QTimer


class ProcessManager:
    """プロセス管理クラス"""
    
    def __init__(self, name, command):
        self.name = name
        self.command = command
        self.process = None
        self.pid_file = f"/tmp/sirius_launcher_{name}.pid"
        self.tab_title = name
        self.script_path = None
        self.pid_file_content = None
    
    def launch(self):
        """コマンドを新しいターミナルタブで起動"""
        try:
            # 一時スクリプトファイルを作成
            script_fd, self.script_path = tempfile.mkstemp(suffix='.sh', prefix='sirius_launcher_')
            
            with os.fdopen(script_fd, 'w') as f:
                # rcfile used by `bash --rcfile` — ensure common shell env is available
                f.write('#!/bin/bash\n')
                # Try to source user's ~/.bashrc so any exported vars are loaded
                f.write('if [ -f "$HOME/.bashrc" ]; then\n')
                f.write('  # shellcheck disable=SC1090\n')
                f.write('  source "$HOME/.bashrc" >/dev/null 2>&1\n')
                f.write('fi\n')
                # If the launcher process had ROS_DOMAIN_ID set, re-export it explicitly
                ros_domain = os.environ.get('ROS_DOMAIN_ID')
                if ros_domain:
                    # quote in case of spaces or other chars
                    f.write(f'export ROS_DOMAIN_ID="{ros_domain}"\n')

                f.write(f'echo $BASHPID > {self.pid_file}\n')
                f.write(f'export PS1="[{self.tab_title}] $ "\n')
                f.write(f'echo -ne "\\033]0;{self.tab_title}\\007"\n')
                f.write(f'{self.command}\n')
            
            os.chmod(self.script_path, 0o755)
            
            # Terminatorで新しいタブを開く
            shell_cmd = f'terminator --new-tab -e "bash --rcfile {self.script_path}"'
            self.process = subprocess.Popen(shell_cmd, shell=True, env=os.environ.copy())
            
            print(f"起動: {self.name} (Terminatorタブ: {self.tab_title})")
            return True
            
        except Exception as e:
            print(f"起動エラー: {e}")
            return False
    
    def load_pid(self, retry_count=0):
        """PIDファイルからプロセスIDを読み込む"""
        try:
            if os.path.exists(self.pid_file):
                with open(self.pid_file, 'r') as f:
                    pid_str = f.read().strip()
                    if not pid_str:
                        return None if retry_count >= 10 else "retry"
                    
                    pid = int(pid_str)
                    if psutil.pid_exists(pid):
                        try:
                            proc = psutil.Process(pid)
                            children = proc.children(recursive=True)
                            if children or retry_count >= 10:
                                self.pid_file_content = pid
                                print(f"プロセス検出: {self.name} (Bash PID: {pid}, 子プロセス: {len(children)}個)")
                                return pid
                            return "retry"
                        except psutil.NoSuchProcess:
                            if os.path.exists(self.pid_file):
                                os.remove(self.pid_file)
                    else:
                        if os.path.exists(self.pid_file):
                            os.remove(self.pid_file)
            else:
                return "retry" if retry_count < 10 else None
        except Exception as e:
            print(f"PID読み込みエラー ({self.name}): {e}")
        return None
    
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
            if not self.pid_file_content:
                print(f"停止対象なし: {self.name}")
                return False
            
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
                    continue
                try:
                    if psutil.pid_exists(pid):
                        proc = psutil.Process(pid)
                        proc.terminate()
                        print(f"  terminate: PID {pid} ({proc.name()})")
                except (psutil.NoSuchProcess, psutil.AccessDenied) as e:
                    print(f"  スキップ: PID {pid} - {e}")
            
            # PIDファイルを削除
            if os.path.exists(self.pid_file):
                os.remove(self.pid_file)
            
            # 一時スクリプトファイルを削除
            if self.script_path and os.path.exists(self.script_path):
                try:
                    os.remove(self.script_path)
                except:
                    pass
            
            print(f"停止: {self.name} (Bash PID: {self.pid_file_content})")
            
            result = {
                'pids': pids,
                'terminator_pid': terminator_pid
            }
            
            self.pid_file_content = None
            return result
            
        except Exception as e:
            print(f"停止エラー: {e}")
            return False
    
    def force_kill_tree(self, pids, terminator_pid=None):
        """プロセスツリーを強制終了"""
        killed_count = 0
        for pid in reversed(pids):
            if pid == terminator_pid:
                continue
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
                            return True
        except Exception as e:
            print(f"タブを閉じるエラー: {e}")
        return False
    
    def is_running(self):
        """プロセスが実行中かチェック"""
        if self.pid_file_content and psutil.pid_exists(self.pid_file_content):
            return True
        else:
            if self.pid_file_content:
                self.pid_file_content = None
                if os.path.exists(self.pid_file):
                    os.remove(self.pid_file)
            return False
