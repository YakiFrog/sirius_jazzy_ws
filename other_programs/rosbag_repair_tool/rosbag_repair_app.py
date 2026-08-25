#!/usr/bin/env python3
"""Desktop GUI for non-destructive recovery of interrupted ROS 2 MCAP bags."""

from pathlib import Path
import os
import signal
import subprocess
import sys

from PySide6.QtCore import QProcess, Qt, QUrl
from PySide6.QtGui import QDesktopServices, QFont
from PySide6.QtWidgets import (
    QApplication,
    QFileDialog,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QProgressBar,
    QPushButton,
    QPlainTextEdit,
    QVBoxLayout,
    QWidget,
)

from rosbag_repair_core import format_bytes, inspect_bag, next_recovered_path


APP_DIR = Path(__file__).resolve().parent
WORKSPACE = APP_DIR.parents[1]
RECOVERY_SCRIPT = WORKSPACE / 'bash/startup_bash/recover_mcap_rosbag.sh'


class RosbagRepairWindow(QMainWindow):
    """Select, inspect, and recover one interrupted MCAP bag."""

    def __init__(self, initial_path: str = ''):
        super().__init__()
        self.setWindowTitle('Rosbag修復ツール（Foxglove対応）')
        self.resize(820, 620)
        self.process = None
        self.inspection = None
        self.output_path = None
        self.stdout_buffer = ''
        self._build_ui()
        if initial_path:
            self.path_edit.setText(initial_path)
            self.inspect_selected_bag()

    def _build_ui(self):
        central = QWidget()
        layout = QVBoxLayout(central)
        layout.setContentsMargins(18, 18, 18, 18)
        layout.setSpacing(12)

        title = QLabel('正常に閉じられなかったROS 2 bagを修復')
        title.setFont(QFont('', 17, QFont.Weight.Bold))
        layout.addWidget(title)

        description = QLabel(
            '元データは変更しません。完全に読めるMCAPチャンクを別フォルダへ救出し、'
            'metadata.yamlとFoxglove用インデックスを再生成します。'
        )
        description.setWordWrap(True)
        layout.addWidget(description)

        path_row = QHBoxLayout()
        self.path_edit = QLineEdit()
        self.path_edit.setPlaceholderText('/home/.../rosbag2_data/rosbag_xxx')
        browse_button = QPushButton('Rosbagを選択')
        browse_button.clicked.connect(self.browse_bag)
        inspect_button = QPushButton('診断')
        inspect_button.clicked.connect(self.inspect_selected_bag)
        path_row.addWidget(self.path_edit, 1)
        path_row.addWidget(browse_button)
        path_row.addWidget(inspect_button)
        layout.addLayout(path_row)

        status_group = QGroupBox('診断結果')
        status_layout = QGridLayout(status_group)
        self.format_value = QLabel('-')
        self.size_value = QLabel('-')
        self.metadata_value = QLabel('-')
        self.footer_value = QLabel('-')
        self.space_value = QLabel('-')
        self.output_value = QLabel('-')
        self.output_value.setTextInteractionFlags(
            Qt.TextInteractionFlag.TextSelectableByMouse
        )
        rows = (
            ('形式', self.format_value),
            ('データ容量', self.size_value),
            ('metadata.yaml', self.metadata_value),
            ('MCAP終了処理', self.footer_value),
            ('空き容量', self.space_value),
            ('復旧先', self.output_value),
        )
        for row, (label, value) in enumerate(rows):
            status_layout.addWidget(QLabel(label), row, 0)
            status_layout.addWidget(value, row, 1)
        status_layout.setColumnStretch(1, 1)
        layout.addWidget(status_group)

        self.status_label = QLabel('Rosbagを選択してください。')
        self.status_label.setWordWrap(True)
        layout.addWidget(self.status_label)

        self.progress = QProgressBar()
        self.progress.setRange(0, 1)
        self.progress.setValue(0)
        layout.addWidget(self.progress)

        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setPlaceholderText('修復ログがここに表示されます。')
        layout.addWidget(self.log, 1)

        button_row = QHBoxLayout()
        self.repair_button = QPushButton('安全な別フォルダへ修復')
        self.repair_button.setEnabled(False)
        self.repair_button.clicked.connect(self.start_repair)
        self.cancel_button = QPushButton('中止')
        self.cancel_button.setEnabled(False)
        self.cancel_button.clicked.connect(self.cancel_repair)
        self.open_button = QPushButton('復旧先を開く')
        self.open_button.setEnabled(False)
        self.open_button.clicked.connect(self.open_output_folder)
        button_row.addWidget(self.repair_button)
        button_row.addWidget(self.cancel_button)
        button_row.addStretch(1)
        button_row.addWidget(self.open_button)
        layout.addLayout(button_row)

        self.setCentralWidget(central)

    def browse_bag(self):
        """Open a directory picker rooted at the usual rosbag directory."""
        start_dir = self.path_edit.text().strip() or '/home/kotantu-desktop/rosbag2_data'
        selected = QFileDialog.getExistingDirectory(
            self, '修復するRosbagディレクトリを選択', start_dir
        )
        if selected:
            self.path_edit.setText(selected)
            self.inspect_selected_bag()

    def inspect_selected_bag(self):
        """Inspect and present the selected bag without writing anything."""
        self.inspection = inspect_bag(self.path_edit.text().strip())
        result = self.inspection
        self.output_path = None
        self.open_button.setEnabled(False)
        self.format_value.setText(
            f'MCAP（{len(result.mcap_files)}ファイル）'
            if result.mcap_files else '-'
        )
        self.size_value.setText(format_bytes(result.total_bytes))
        self.metadata_value.setText('あり' if result.metadata_exists else 'なし（要修復）')
        self.footer_value.setText(
            '正常' if result.all_files_finalized else '未完了・破損の可能性'
        )
        self.space_value.setText(
            f'{format_bytes(result.available_bytes)} 空き / '
            f'{format_bytes(result.required_bytes)} 必要'
        )
        self.output_value.setText(str(next_recovered_path(result.source)))

        if result.error:
            self.status_label.setText(f'✗ {result.error}')
            self.repair_button.setEnabled(False)
        elif not result.has_enough_space:
            self.status_label.setText(
                '✗ 空き容量が不足しています。元データと同程度の空きを確保してください。'
            )
            self.repair_button.setEnabled(False)
        elif not result.metadata_exists or not result.all_files_finalized:
            self.status_label.setText(
                '⚠ 正常終了していないMCAPです。修復を実行できます。'
            )
            self.repair_button.setEnabled(True)
        else:
            self.status_label.setText(
                '✓ 構造上は正常です。再構築コピーを作ることもできます。'
            )
            self.repair_button.setEnabled(True)

    def start_repair(self):
        """Start the existing non-destructive recovery workflow."""
        if not self.inspection or not self.inspection.can_repair:
            self.inspect_selected_bag()
            if not self.inspection or not self.inspection.can_repair:
                return
        if not RECOVERY_SCRIPT.is_file():
            QMessageBox.critical(
                self, 'エラー', f'復旧処理がありません: {RECOVERY_SCRIPT}'
            )
            return

        self.log.clear()
        self.stdout_buffer = ''
        self.output_path = None
        self.progress.setRange(0, 0)
        self.status_label.setText('修復中です。大きなbagでは数分かかります…')
        self.repair_button.setEnabled(False)
        self.cancel_button.setEnabled(True)
        self.open_button.setEnabled(False)

        self.process = QProcess(self)
        self.process.setProgram('/usr/bin/setsid')
        self.process.setArguments([
            '/bin/bash', str(RECOVERY_SCRIPT), str(self.inspection.source)
        ])
        self.process.readyReadStandardOutput.connect(self._read_stdout)
        self.process.readyReadStandardError.connect(self._read_stderr)
        self.process.finished.connect(self._repair_finished)
        self.process.errorOccurred.connect(self._process_error)
        self.process.start()

    def _append_log(self, text: str):
        if text:
            self.log.appendPlainText(text.rstrip())

    def _read_stdout(self):
        text = bytes(self.process.readAllStandardOutput()).decode('utf-8', errors='replace')
        self.stdout_buffer += text
        self._append_log(text)

    def _read_stderr(self):
        text = bytes(self.process.readAllStandardError()).decode('utf-8', errors='replace')
        self._append_log(text)

    def _process_error(self, _error):
        if self.process:
            self._append_log(f'プロセスエラー: {self.process.errorString()}')

    def _repair_finished(self, exit_code: int, _exit_status):
        self.progress.setRange(0, 1)
        self.progress.setValue(1 if exit_code == 0 else 0)
        self.cancel_button.setEnabled(False)
        self.repair_button.setEnabled(True)

        output_lines = [line.strip() for line in self.stdout_buffer.splitlines() if line.strip()]
        candidate = Path(output_lines[-1]) if output_lines else None
        if exit_code == 0 and candidate and candidate.is_dir():
            self.output_path = candidate
            self.output_value.setText(str(candidate))
            self.open_button.setEnabled(True)
            validation = subprocess.run(
                ['ros2', 'bag', 'info', str(candidate)],
                capture_output=True, text=True, timeout=60, check=False,
            )
            if validation.returncode == 0:
                self._append_log(validation.stdout)
                self.status_label.setText(
                    '✓ 修復成功。Foxgloveで復旧先の.mcapファイルを開けます。'
                )
                QMessageBox.information(
                    self,
                    '修復完了',
                    f'元データを残したまま修復しました。\n\n{candidate}',
                )
            else:
                self._append_log(validation.stderr)
                self.status_label.setText('✗ 出力後のRosbag検証に失敗しました。')
        else:
            self.status_label.setText(
                '✗ 修復に失敗または中止しました。元データは変更されていません。'
            )
        self.process = None

    def cancel_repair(self):
        """Terminate the recovery process group and preserve the source bag."""
        if not self.process or self.process.state() == QProcess.ProcessState.NotRunning:
            return
        answer = QMessageBox.question(
            self,
            '修復を中止',
            '修復を中止しますか？途中の一時出力は削除され、元データは残ります。',
        )
        if answer != QMessageBox.StandardButton.Yes:
            return
        pid = int(self.process.processId())
        try:
            os.killpg(pid, signal.SIGTERM)
        except (ProcessLookupError, PermissionError):
            self.process.terminate()

    def open_output_folder(self):
        """Open the recovered bag directory in the desktop file manager."""
        if self.output_path and self.output_path.is_dir():
            QDesktopServices.openUrl(QUrl.fromLocalFile(str(self.output_path)))

    def closeEvent(self, event):
        if self.process and self.process.state() != QProcess.ProcessState.NotRunning:
            QMessageBox.warning(
                self, '修復中', '修復中です。先に「中止」を押してください。'
            )
            event.ignore()
            return
        event.accept()


def main():
    """Launch the repair application."""
    app = QApplication(sys.argv)
    initial_path = sys.argv[1] if len(sys.argv) > 1 else ''
    window = RosbagRepairWindow(initial_path)
    window.show()
    return app.exec()


if __name__ == '__main__':
    raise SystemExit(main())
