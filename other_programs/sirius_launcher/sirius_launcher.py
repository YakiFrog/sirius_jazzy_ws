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
from ui_components import LaunchButtonUI, MainWindowUI, CollapsibleSection
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
        
        self.has_error = False
        self.running = False
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
        """コマンドを新しいターミナルタブで起動。既に起動している場合はそのタブを表示"""
        if self.process_manager.is_running():
            print(f"既に起動しています。タブをフォーカスします: {self.name}")
            import shutil
            tools_installed = shutil.which('wmctrl') is not None and shutil.which('xdotool') is not None
            
            if not self.process_manager.focus_terminator_tab():
                if not tools_installed:
                    QMessageBox.information(
                        self, 
                        "情報", 
                        f"「{self.name}」は既に起動しています。\n\n"
                        "※開いているターミナルタブに自動で切り替えるには、wmctrl と xdotool のインストールが必要です。\n"
                        "以下のコマンドを実行してください：\n"
                        "sudo apt install wmctrl xdotool -y"
                    )
                else:
                    QMessageBox.warning(
                        self,
                        "警告",
                        f"「{self.name}」のターミナルタブをフォーカスできませんでした。\n"
                        "タブが手動で閉じられたか、名前が変更された可能性があります。"
                    )
            return
            
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
        self.has_error = self.process_manager.check_for_errors() if is_running else False
        self.running = is_running
        
        self.update_status(is_running, self.has_error)
        
        # メインウィンドウに通知してタブの状態を更新させる
        main_win = self.window()
        if hasattr(main_win, 'update_tab_error_status'):
            main_win.update_tab_error_status()

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
        self.original_tab_names = {} # index -> name
        self.setup_ui()
        self.load_aliases()
    
    def setup_ui(self):
        """UIのセットアップ"""
        self.preset_layout, self.tab_layouts, self.tab_widget, self.reload_btn = MainWindowUI.setup_ui(self)
        self.reload_btn.clicked.connect(self.reload_launcher)
        if hasattr(self, 'preset_edit_btn'):
            self.preset_edit_btn.clicked.connect(self.edit_presets)

    def edit_presets(self):
        """プリセット編集ダイアログを開き、保存後に再読込する"""
        from preset_editor import PresetEditorDialog, save_presets_to_file
        alias_file = Path.home() / "sirius_jazzy_ws" / "bash" / "bash_alias2.sh"
        dialog = PresetEditorDialog(self.presets, list(self.button_map.keys()), self)
        if dialog.exec():
            save_presets_to_file(str(alias_file), dialog.presets)
            print("プリセットを保存しました。再読み込みします。")
            self.reload_launcher()

    def add_group(self, title, tab_name=None, description=""):
        """グループボックスを追加（タブ対応）"""
        group, group_layout = MainWindowUI.create_group(title, description)
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

        groups, presets, group_descriptions = parse_bash_aliases(
            str(alias_file), include_group_descriptions=True
        )
        self.presets = presets

        if not groups:
            QMessageBox.warning(self, "警告", "エイリアスが見つかりませんでした。")
            return

        # プリセットボタンを作成
        for preset_name, items in presets:
            self.add_preset_button(preset_name, items)
        # 上揃え（余った縦スペースは下端へ）
        self.preset_layout.addStretch(1)

        # タブ名リスト（ui_components.pyのデフォルトと合わせる）
        tab_names_list = [
            "センサー・ハードウェア",
            "シミュレーション",
            "ユーティリティ",
            "ナビゲーション",
            "Pythonスクリプト",
            "Sirius Ear関連",
            "リアル実験",
            "オフライン・マッピング",
        ]
        for i, name in enumerate(tab_names_list):
            self.original_tab_names[i] = name

        # 通常のボタンを作成（タブごとにグループ追加）
        for i, (group_name, aliases) in enumerate(groups.items()):
            if aliases:
                tab_name = tab_names_list[i] if i < len(tab_names_list) else tab_names_list[0]
                group_layout, group_widget = self.add_group(
                    group_name,
                    tab_name,
                    group_descriptions.get(group_name, ""),
                )
                current_section = None
                seen_subgroup = None
                for item in aliases:
                    alias_name, command, description = item[0], item[1], item[2]
                    subgroup = item[3] if len(item) > 3 else ""
                    if subgroup:
                        if subgroup != seen_subgroup:
                            current_section = CollapsibleSection(subgroup, expanded=False)
                            group_layout.addWidget(current_section)
                            seen_subgroup = subgroup
                        self.add_button(current_section.content_layout, alias_name, command,
                                        description, group_widget)
                    else:
                        current_section = None
                        self.add_button(group_layout, alias_name, command, description, group_widget)

        # 各タブのレイアウトにストレッチ追加
        for layout in self.tab_layouts.values():
            layout.addStretch()
            
    def clear_layout(self, layout):
        """レイアウト内のウィジェットを再帰的に削除"""
        while layout.count():
            child = layout.takeAt(0)
            if child.widget():
                child.widget().deleteLater()
            elif child.layout():
                self.clear_layout(child.layout())
                
    def reload_launcher(self):
        """エイリアスファイルを再パースしてGUIを再構築（実行中のプロセスはそのまま維持）"""
        print("🔄 エイリアス設定を再読み込み中...")
        
        # 1. 内部のボタンマップ等をクリア
        self.buttons = []
        self.button_map = {}
        self.presets = []
        
        # 2. プリセットボタンをUIから削除
        self.clear_layout(self.preset_layout)
        
        # 3. 各タブのグループボックスなどのUI要素を削除
        for layout in self.tab_layouts.values():
            self.clear_layout(layout)
            
        # 4. 再ロードと再配置
        self.load_aliases()
        print("✅ 再読み込み完了。")
    
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
            # 親を辿ってTabのインデックスを探す（ScrollArea導入に対応）
            parent = group_widget.parentWidget()
            while parent:
                tab_index = self.tab_widget.indexOf(parent)
                if tab_index >= 0:
                    break
                parent = parent.parentWidget()
        button = LaunchButton(name, command, description, tab_widget=self.tab_widget, tab_index=tab_index)
        layout.addWidget(button)
        self.buttons.append(button)
        self.button_map[name] = button

    def update_tab_error_status(self):
        """全てのタブのエラー状況をスキャンして表示を更新"""
        tab_errors = {} # index -> bool
        
        # 各タブにエラーがあるかチェック
        for button in self.buttons:
            if button.tab_index is not None:
                if button.tab_index not in tab_errors:
                    tab_errors[button.tab_index] = False
                if button.has_error:
                    tab_errors[button.tab_index] = True
        
        # タブの表示（テキスト）を更新
        for idx, original_name in self.original_tab_names.items():
            if idx < self.tab_widget.count():
                has_error = tab_errors.get(idx, False)
                current_text = self.tab_widget.tabText(idx)
                
                if has_error:
                    new_text = "⚠️ " + original_name
                    if current_text != new_text:
                        self.tab_widget.setTabText(idx, new_text)
                else:
                    if current_text != original_name:
                        self.tab_widget.setTabText(idx, original_name)

        # タブ右上の「起動中プロセス数」バッジを更新
        running_counts = {}
        for button in self.buttons:
            if getattr(button, 'running', False) and button.tab_index is not None:
                running_counts[button.tab_index] = running_counts.get(button.tab_index, 0) + 1
        for idx, badge in getattr(self, 'tab_badges', {}).items():
            count = running_counts.get(idx, 0)
            if count > 0:
                badge.setText(str(count))
                badge.setVisible(True)
            else:
                badge.setText("")
                badge.setVisible(False)


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
