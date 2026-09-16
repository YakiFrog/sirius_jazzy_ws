#!/usr/bin/env python3
"""Sirius Launcher - SAM3 設定ダイアログ。

ローカルの SAM3 サーバ (既定 http://localhost:8080) のライブ設定を読み書きする。
  - prompt / confidence threshold / color_mode
  - クラス別しきい値 (class_thresholds)
  - SAM3サーバ(Docker)の起動、Web UI を開く
"""

import os
import subprocess
import webbrowser

import requests
from PySide6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit, QPushButton,
    QDoubleSpinBox, QComboBox, QTableWidget, QTableWidgetItem, QMessageBox,
    QHeaderView, QAbstractItemView,
)

SAM3_SERVER_DIR = os.path.expanduser("~/sam3_zed_server")
SAM3_CONTAINER = "sam3_zed_container"


class Sam3SettingsDialog(QDialog):
    def __init__(self, server="http://localhost:8080", parent=None):
        super().__init__(parent)
        self.server = server.rstrip("/")
        self.setWindowTitle("SAM3 設定")
        self.resize(620, 560)
        self._build_ui()
        self.refresh()

    def _build_ui(self):
        root = QVBoxLayout(self)

        self.status = QLabel("未接続")
        self.status.setStyleSheet("color: #6c757d;")
        root.addWidget(self.status)

        row = QHBoxLayout()
        row.addWidget(QLabel("prompt:"))
        self.prompt_edit = QLineEdit()
        self.prompt_edit.setPlaceholderText("例: grass, tactile paving, roadway, sidewalk")
        row.addWidget(self.prompt_edit, 1)
        root.addLayout(row)

        row2 = QHBoxLayout()
        row2.addWidget(QLabel("confidence threshold:"))
        self.threshold_spin = QDoubleSpinBox()
        self.threshold_spin.setRange(0.0, 1.0)
        self.threshold_spin.setSingleStep(0.05)
        self.threshold_spin.setDecimals(2)
        self.threshold_spin.setValue(0.5)
        row2.addWidget(self.threshold_spin)
        row2.addSpacing(20)
        row2.addWidget(QLabel("color_mode:"))
        self.color_mode_combo = QComboBox()
        self.color_mode_combo.addItems(["semantic", "real"])
        row2.addWidget(self.color_mode_combo)
        row2.addStretch(1)
        root.addLayout(row2)

        root.addWidget(QLabel("クラス別しきい値 (class_thresholds):"))
        self.table = QTableWidget(0, 2)
        self.table.setHorizontalHeaderLabels(["class", "threshold"])
        self.table.horizontalHeader().setSectionResizeMode(0, QHeaderView.Stretch)
        self.table.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeToContents)
        self.table.setSelectionBehavior(QAbstractItemView.SelectRows)
        root.addWidget(self.table, 1)

        add_row = QHBoxLayout()
        add_btn = QPushButton("行を追加")
        add_btn.clicked.connect(lambda: self._add_row("", self.threshold_spin.value()))
        del_btn = QPushButton("選択行を削除")
        del_btn.clicked.connect(self._remove_row)
        add_row.addWidget(add_btn)
        add_row.addWidget(del_btn)
        add_row.addStretch(1)
        root.addLayout(add_row)

        btns = QHBoxLayout()
        refresh_btn = QPushButton("🔄 再取得")
        refresh_btn.clicked.connect(self.refresh)
        apply_btn = QPushButton("適用")
        apply_btn.setStyleSheet("background-color:#28a745;color:white;font-weight:bold;padding:6px 16px;")
        apply_btn.clicked.connect(self.apply)
        start_btn = QPushButton("SAM3サーバー起動")
        start_btn.clicked.connect(self.start_server)
        web_btn = QPushButton("Web UIを開く")
        web_btn.clicked.connect(lambda: webbrowser.open(self.server + "/"))
        close_btn = QPushButton("閉じる")
        close_btn.clicked.connect(self.reject)
        for b in (refresh_btn, apply_btn, start_btn, web_btn):
            btns.addWidget(b)
        btns.addStretch(1)
        btns.addWidget(close_btn)
        root.addLayout(btns)

    # --- サーバ通信 ---
    def _get_json(self, path):
        resp = requests.get(self.server + path, timeout=2.0)
        resp.raise_for_status()
        return resp.json()

    def _post_json(self, path, payload):
        resp = requests.post(self.server + path, json=payload, timeout=2.0)
        resp.raise_for_status()
        return resp.json()

    def refresh(self):
        try:
            state = self._get_json("/debug_state")
        except Exception as error:
            self.status.setText(f"未接続（{self.server} が応答しません）: {error}")
            self.status.setStyleSheet("color: #dc3545;")
            return
        settings = state.get("sam3_settings", {})
        self.prompt_edit.setText(settings.get("prompt", ""))
        try:
            self.threshold_spin.setValue(float(settings.get("confidence_threshold", 0.5)))
        except (TypeError, ValueError):
            pass
        mode = settings.get("color_mode", "semantic")
        idx = self.color_mode_combo.findText(mode)
        self.color_mode_combo.setCurrentIndex(idx if idx >= 0 else 0)

        self.table.setRowCount(0)
        for name, value in (settings.get("class_thresholds") or {}).items():
            try:
                self._add_row(str(name), float(value))
            except (TypeError, ValueError):
                self._add_row(str(name), self.threshold_spin.value())
        self.status.setText(f"接続OK: {self.server}")
        self.status.setStyleSheet("color: #28a745;")

    def _add_row(self, name, threshold):
        row = self.table.rowCount()
        self.table.insertRow(row)
        self.table.setItem(row, 0, QTableWidgetItem(str(name)))
        self.table.setItem(row, 1, QTableWidgetItem(str(threshold)))

    def _remove_row(self):
        row = self.table.currentRow()
        if row >= 0:
            self.table.removeRow(row)

    def apply(self):
        classes = {}
        for row in range(self.table.rowCount()):
            name_item = self.table.item(row, 0)
            value_item = self.table.item(row, 1)
            if name_item is None or not name_item.text().strip():
                continue
            try:
                classes[name_item.text().strip()] = float(value_item.text())
            except (AttributeError, ValueError):
                continue
        try:
            self._post_json("/prompt", {"prompt": self.prompt_edit.text()})
            self._post_json("/threshold", {"threshold": float(self.threshold_spin.value())})
            self._post_json("/color_mode", {"mode": self.color_mode_combo.currentText()})
            self._post_json("/class_thresholds",
                            {"default": float(self.threshold_spin.value()), "classes": classes})
        except Exception as error:
            QMessageBox.critical(self, "エラー", f"適用に失敗しました: {error}")
            return
        self.status.setText("適用しました（※THETAノードは起動時にprompt/thresholdを再送します）")
        self.status.setStyleSheet("color: #0b5ed7;")

    def start_server(self):
        if subprocess.run(["docker", "ps", "--format", "{{.Names}}"],
                          capture_output=True, text=True).stdout.find(SAM3_CONTAINER) >= 0:
            self.status.setText("SAM3サーバーは既に起動しています")
            return
        subprocess.Popen(["docker", "compose", "up", "-d", "sam3-zed-merged"], cwd=SAM3_SERVER_DIR)
        self.status.setText("SAM3サーバーを起動中…（起動後に『再取得』）")
