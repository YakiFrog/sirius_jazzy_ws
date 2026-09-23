#!/usr/bin/env bash
# Sirius Launcher の SAM3 設定ダイアログ（theta_sam3.yaml を編集・保存）を単体起動する。
set -eu

LAUNCHER_DIR="${HOME}/sirius_jazzy_ws/other_programs/sirius_launcher"

if [ ! -f "$LAUNCHER_DIR/sam3_settings_dialog.py" ]; then
    echo "エラー: SAM3設定ダイアログがありません: $LAUNCHER_DIR"
    exit 1
fi

cd "$LAUNCHER_DIR"
exec python3 -c '
import sys
from PySide6.QtWidgets import QApplication
from sam3_settings_dialog import Sam3SettingsDialog
app = QApplication(sys.argv)
dialog = Sam3SettingsDialog()
dialog.exec()
'
