#!/usr/bin/env python3
"""Sirius Launcher - プリセット編集ダイアログと保存処理。

プリセットは bash_alias2.sh の冒頭に
    # PRESET: 名前
    # PRESET_ITEMS: item1,item2,...
の形式で保存されている。このモジュールはGUIで編集し、その領域だけを書き換える。
"""

import os
import shutil

from PySide6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QLabel, QListWidget, QPushButton,
    QLineEdit, QInputDialog, QMessageBox,
)
from PySide6.QtCore import Qt


class PresetEditorDialog(QDialog):
    """プリセットを追加・編集・並べ替えするダイアログ。"""

    def __init__(self, presets, all_aliases, parent=None):
        super().__init__(parent)
        self.all_aliases = list(all_aliases)
        self.presets = [[name, list(items)] for name, items in presets]
        self.current = -1

        self.setWindowTitle("プリセット編集")
        self.resize(1040, 620)
        root = QVBoxLayout(self)

        cols = QHBoxLayout()

        # 左: プリセット一覧
        pcol = QVBoxLayout()
        pcol.addWidget(QLabel("プリセット"))
        self.preset_list = QListWidget()
        pcol.addWidget(self.preset_list)
        prow = QHBoxLayout()
        for text, slot in (("新規", self.add_preset), ("複製", self.dup_preset),
                           ("名前変更", self.rename_preset), ("削除", self.del_preset)):
            btn = QPushButton(text)
            btn.clicked.connect(slot)
            prow.addWidget(btn)
        pcol.addLayout(prow)
        cols.addLayout(pcol, 2)

        # 中: このプリセットで起動する項目（順番）
        scol = QVBoxLayout()
        scol.addWidget(QLabel("このプリセットで起動（上から順）"))
        self.selected_list = QListWidget()
        scol.addWidget(self.selected_list)
        srow = QHBoxLayout()
        for text, slot in (("↑", lambda: self.move_item(-1)),
                           ("↓", lambda: self.move_item(1)),
                           ("この項目を削除", self.remove_item)):
            btn = QPushButton(text)
            btn.clicked.connect(slot)
            srow.addWidget(btn)
        scol.addLayout(srow)
        cols.addLayout(scol, 2)

        # 右: 利用可能なコマンド
        acol = QVBoxLayout()
        acol.addWidget(QLabel("利用可能なコマンド（ダブルクリックで追加）"))
        self.filter_edit = QLineEdit()
        self.filter_edit.setPlaceholderText("絞り込み")
        self.filter_edit.textChanged.connect(self.refill_available)
        acol.addWidget(self.filter_edit)
        self.available_list = QListWidget()
        self.available_list.itemDoubleClicked.connect(lambda _item: self.add_item())
        acol.addWidget(self.available_list)
        add_btn = QPushButton("← 選択した項目をプリセットに追加")
        add_btn.clicked.connect(self.add_item)
        acol.addWidget(add_btn)
        cols.addLayout(acol, 3)

        root.addLayout(cols)

        btns = QHBoxLayout()
        btns.addStretch(1)
        save_btn = QPushButton("保存")
        save_btn.setStyleSheet("background-color: #28a745; color: white; font-weight: bold; padding: 6px 16px;")
        save_btn.clicked.connect(self.on_save)
        cancel_btn = QPushButton("キャンセル")
        cancel_btn.clicked.connect(self.reject)
        btns.addWidget(save_btn)
        btns.addWidget(cancel_btn)
        root.addLayout(btns)

        self.preset_list.currentRowChanged.connect(self.on_preset_changed)
        self.refill_available()
        self._refresh_preset_list(select=0 if self.presets else None)

    # --- 一覧の再構築 ---
    def _refresh_preset_list(self, select=None):
        self.preset_list.blockSignals(True)
        self.preset_list.clear()
        for name, _items in self.presets:
            self.preset_list.addItem(name)
        self.preset_list.blockSignals(False)
        if select is not None and 0 <= select < len(self.presets):
            self.preset_list.setCurrentRow(select)
        elif self.presets:
            self.preset_list.setCurrentRow(0)
        elif hasattr(self, "selected_list"):
            self.selected_list.clear()

    def on_preset_changed(self, row):
        self._commit_current()
        self.current = row
        self.selected_list.clear()
        if 0 <= row < len(self.presets):
            for item in self.presets[row][1]:
                self.selected_list.addItem(item)

    def _commit_current(self):
        if 0 <= self.current < len(self.presets):
            self.presets[self.current][1] = [
                self.selected_list.item(i).text() for i in range(self.selected_list.count())
            ]

    def refill_available(self):
        query = self.filter_edit.text().strip().lower()
        self.available_list.clear()
        for alias in self.all_aliases:
            if not query or query in alias.lower():
                self.available_list.addItem(alias)

    # --- 項目操作 ---
    def add_item(self):
        item = self.available_list.currentItem()
        if item is not None:
            self.selected_list.addItem(item.text())
            self.selected_list.setCurrentRow(self.selected_list.count() - 1)

    def remove_item(self):
        row = self.selected_list.currentRow()
        if row >= 0:
            self.selected_list.takeItem(row)

    def move_item(self, delta):
        row = self.selected_list.currentRow()
        new_row = row + delta
        if row < 0 or new_row < 0 or new_row >= self.selected_list.count():
            return
        item = self.selected_list.takeItem(row)
        self.selected_list.insertItem(new_row, item)
        self.selected_list.setCurrentRow(new_row)

    # --- プリセット操作 ---
    def add_preset(self):
        self._commit_current()
        self.presets.append([f"新しいプリセット {len(self.presets) + 1}", []])
        self._refresh_preset_list(select=len(self.presets) - 1)

    def dup_preset(self):
        self._commit_current()
        row = self.preset_list.currentRow()
        if row < 0:
            return
        self.presets.append([self.presets[row][0] + " のコピー", list(self.presets[row][1])])
        self._refresh_preset_list(select=len(self.presets) - 1)

    def rename_preset(self):
        row = self.preset_list.currentRow()
        if row < 0:
            return
        name, ok = QInputDialog.getText(self, "名前変更", "新しい名前:", text=self.presets[row][0])
        if ok and name.strip():
            self.presets[row][0] = name.strip()
            item = self.preset_list.item(row)
            if item is not None:
                item.setText(self.presets[row][0])

    def del_preset(self):
        row = self.preset_list.currentRow()
        if row < 0:
            return
        confirm = QMessageBox.question(self, "削除確認",
                                       f"プリセット「{self.presets[row][0]}」を削除しますか?")
        if confirm != QMessageBox.Yes:
            return
        del self.presets[row]
        self.current = -1
        self._refresh_preset_list(select=0 if self.presets else None)

    def on_save(self):
        self._commit_current()
        self.accept()


def save_presets_to_file(path, presets, backup=True):
    """bash_alias2.sh の先頭にあるプリセット領域だけを書き換える（以降は保持）。"""
    if not os.path.exists(path):
        raise FileNotFoundError(path)
    with open(path, encoding="utf-8") as f:
        lines = f.read().split("\n")

    head_index = len(lines)
    for i, line in enumerate(lines):
        stripped = line.strip()
        if stripped.startswith("# TAB:") or stripped.startswith("# GROUP:"):
            head_index = i
            break

    tail = lines[head_index:]
    block = []
    for name, items in presets:
        block.append(f"# PRESET: {name}")
        block.append(f"# PRESET_ITEMS: {','.join(items)}")
        block.append("")

    new_text = "\n".join(block + tail)
    if backup:
        shutil.copy(path, path + ".bak")
    with open(path, "w", encoding="utf-8") as f:
        f.write(new_text)
