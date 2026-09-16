#!/usr/bin/env python3
"""Sirius Launcher - SAM3 設定ダイアログ。

THETA マッピング用 SAM3 設定を **共有YAML** に保存する（theta_sam3_perspective_node が
起動時に読み、SAM3サーバへ送信する）。「サーバーに適用」で起動中サーバへも即時反映する。
"""

import os
import subprocess
import webbrowser

import yaml
import requests
from PySide6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit, QPushButton,
    QDoubleSpinBox, QTableWidget, QTableWidgetItem, QMessageBox,
    QHeaderView, QAbstractItemView, QFileDialog,
)

DEFAULT_CONFIG = os.path.expanduser(
    "~/sirius_jazzy_ws/src/sirius/sirius_navigation/config/theta_sam3.yaml")
SAM3_SERVER_DIR = os.path.expanduser("~/sam3_zed_server")
SAM3_CONTAINER = "sam3_zed_container"


class Sam3SettingsDialog(QDialog):
    def __init__(self, config_path=DEFAULT_CONFIG, server="http://localhost:8080", parent=None):
        super().__init__(parent)
        self.config_path = config_path
        self.server = server.rstrip("/")
        self.setWindowTitle("SAM3 設定（THETAマッピング用）")
        self.resize(720, 600)
        self._build_ui()
        self.load_config()

    def _build_ui(self):
        root = QVBoxLayout(self)

        path_row = QHBoxLayout()
        path_row.addWidget(QLabel("設定ファイル:"))
        self.path_edit = QLineEdit(self.config_path)
        path_row.addWidget(self.path_edit, 1)
        browse = QPushButton("参照")
        browse.clicked.connect(self._browse)
        path_row.addWidget(browse)
        root.addLayout(path_row)

        self.status = QLabel("")
        self.status.setStyleSheet("color: #6c757d;")
        root.addWidget(self.status)

        row = QHBoxLayout()
        row.addWidget(QLabel("prompt:"))
        self.prompt_edit = QLineEdit()
        self.prompt_edit.setPlaceholderText("例: grass, tactile paving, line-type tactile paving")
        row.addWidget(self.prompt_edit, 1)
        root.addLayout(row)

        row2 = QHBoxLayout()
        row2.addWidget(QLabel("threshold:"))
        self.threshold_spin = self._make_spin(0.3)
        row2.addWidget(self.threshold_spin)
        row2.addSpacing(16)
        row2.addWidget(QLabel("score_min:"))
        self.score_min_spin = self._make_spin(0.3)
        row2.addWidget(self.score_min_spin)
        row2.addStretch(1)
        root.addLayout(row2)

        root.addWidget(QLabel("classes（クラス名 / 予約ID / 色(r,g,b) / しきい値）:"))
        self.table = QTableWidget(0, 4)
        self.table.setHorizontalHeaderLabels(["class", "id", "color (r,g,b)", "threshold"])
        self.table.horizontalHeader().setSectionResizeMode(0, QHeaderView.Stretch)
        for col in (1, 2, 3):
            self.table.horizontalHeader().setSectionResizeMode(col, QHeaderView.ResizeToContents)
        self.table.setSelectionBehavior(QAbstractItemView.SelectRows)
        root.addWidget(self.table, 1)

        tbl_btns = QHBoxLayout()
        add_btn = QPushButton("行を追加")
        add_btn.clicked.connect(lambda: self._add_row("", 3, [0, 255, 0], self.threshold_spin.value()))
        del_btn = QPushButton("選択行を削除")
        del_btn.clicked.connect(self._remove_row)
        tbl_btns.addWidget(add_btn)
        tbl_btns.addWidget(del_btn)
        tbl_btns.addStretch(1)
        root.addLayout(tbl_btns)

        btns = QHBoxLayout()
        load_btn = QPushButton("🔄 読込")
        load_btn.clicked.connect(self.load_config)
        save_btn = QPushButton("💾 保存")
        save_btn.setStyleSheet("background-color:#28a745;color:white;font-weight:bold;padding:6px 16px;")
        save_btn.clicked.connect(self.save_config)
        apply_btn = QPushButton("サーバーに適用")
        apply_btn.clicked.connect(self.apply_to_server)
        start_btn = QPushButton("サーバー起動")
        start_btn.clicked.connect(self.start_server)
        web_btn = QPushButton("Web UI")
        web_btn.clicked.connect(lambda: webbrowser.open(self.server + "/"))
        close_btn = QPushButton("閉じる")
        close_btn.clicked.connect(self.reject)
        for b in (load_btn, save_btn, apply_btn, start_btn, web_btn):
            btns.addWidget(b)
        btns.addStretch(1)
        btns.addWidget(close_btn)
        root.addLayout(btns)

    @staticmethod
    def _make_spin(value):
        from PySide6.QtWidgets import QDoubleSpinBox
        spin = QDoubleSpinBox()
        spin.setRange(0.0, 1.0)
        spin.setSingleStep(0.05)
        spin.setDecimals(2)
        spin.setValue(value)
        return spin

    def _browse(self):
        path, _ = QFileDialog.getSaveFileName(self, "設定YAML", self.path_edit.text(),
                                              "YAML (*.yaml *.yml)")
        if path:
            self.path_edit.setText(path)

    def _add_row(self, name, class_id, color, threshold):
        row = self.table.rowCount()
        self.table.insertRow(row)
        self.table.setItem(row, 0, QTableWidgetItem(str(name)))
        self.table.setItem(row, 1, QTableWidgetItem(str(class_id)))
        self.table.setItem(row, 2, QTableWidgetItem(",".join(str(int(c)) for c in color)))
        self.table.setItem(row, 3, QTableWidgetItem(str(threshold)))

    def _remove_row(self):
        row = self.table.currentRow()
        if row >= 0:
            self.table.removeRow(row)

    def load_config(self):
        path = self.path_edit.text().strip()
        if not os.path.exists(path):
            self.status.setText(f"設定ファイルが無いので既定を表示: {path}")
            self.status.setStyleSheet("color: #6c757d;")
            return
        try:
            with open(path, encoding="utf-8") as stream:
                cfg = yaml.safe_load(stream) or {}
        except Exception as error:
            self.status.setText(f"読込失敗: {error}")
            self.status.setStyleSheet("color: #dc3545;")
            return
        self.prompt_edit.setText(str(cfg.get("prompt", "")))
        self.threshold_spin.setValue(float(cfg.get("threshold", 0.3) or 0.3))
        self.score_min_spin.setValue(float(cfg.get("score_min", 0.3) or 0.3))
        class_thresholds = cfg.get("class_thresholds") or {}
        self.table.setRowCount(0)
        for name, spec in (cfg.get("classes") or {}).items():
            spec = spec or {}
            color = spec.get("color", [255, 255, 255])
            self._add_row(name, spec.get("id", 0), color,
                          class_thresholds.get(name, self.threshold_spin.value()))
        self.status.setText(f"読込ました: {path}")
        self.status.setStyleSheet("color: #28a745;")

    def _collect(self):
        classes = {}
        class_thresholds = {}
        for row in range(self.table.rowCount()):
            name_item = self.table.item(row, 0)
            if name_item is None or not name_item.text().strip():
                continue
            name = name_item.text().strip()
            try:
                class_id = int(float(self.table.item(row, 1).text()))
            except (AttributeError, ValueError):
                class_id = 0
            try:
                color = [int(float(x)) for x in self.table.item(row, 2).text().split(",")][:3]
                while len(color) < 3:
                    color.append(0)
            except (AttributeError, ValueError):
                color = [255, 255, 255]
            classes[name] = {"id": class_id, "color": color}
            try:
                class_thresholds[name] = float(self.table.item(row, 3).text())
            except (AttributeError, ValueError):
                pass
        return {
            "prompt": self.prompt_edit.text(),
            "threshold": float(self.threshold_spin.value()),
            "score_min": float(self.score_min_spin.value()),
            "classes": classes,
            "class_thresholds": class_thresholds,
        }

    def save_config(self):
        path = self.path_edit.text().strip()
        try:
            with open(path, "w", encoding="utf-8") as stream:
                yaml.safe_dump(self._collect(), stream, sort_keys=False, allow_unicode=True)
        except Exception as error:
            QMessageBox.critical(self, "エラー", f"保存に失敗しました: {error}")
            return False
        self.status.setText(f"保存しました: {path}（次回のTHETAマッピングから反映）")
        self.status.setStyleSheet("color: #28a745;")
        return True

    def apply_to_server(self):
        cfg = self._collect()
        try:
            requests.post(self.server + "/prompt", json={"prompt": cfg["prompt"]}, timeout=2.0)
            requests.post(self.server + "/threshold", json={"threshold": cfg["threshold"]}, timeout=2.0)
            requests.post(self.server + "/class_registry", json={"classes": cfg["classes"]}, timeout=2.0)
            if cfg["class_thresholds"]:
                requests.post(self.server + "/class_thresholds",
                              json={"default": cfg["threshold"], "classes": cfg["class_thresholds"]},
                              timeout=2.0)
        except Exception as error:
            self.status.setText(f"サーバー未接続（YAML保存のみ）: {error}")
            self.status.setStyleSheet("color: #dc3545;")
            return
        self.status.setText("サーバーに適用しました（その実行中のみ有効）")
        self.status.setStyleSheet("color: #0b5ed7;")

    def start_server(self):
        names = subprocess.run(["docker", "ps", "--format", "{{.Names}}"],
                               capture_output=True, text=True).stdout
        if SAM3_CONTAINER in names:
            self.status.setText("SAM3サーバーは既に起動しています")
            return
        subprocess.Popen(["docker", "compose", "up", "-d", "sam3-zed-merged"], cwd=SAM3_SERVER_DIR)
        self.status.setText("SAM3サーバーを起動中…（起動後に『サーバーに適用』）")
