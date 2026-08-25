#!/usr/bin/env python3
"""PySide6 desktop UI for selective SAM3 semantic-map repair."""

from __future__ import annotations

import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.request

from PySide6.QtCore import QProcess, QSettings, QThread, QTimer, Qt, QUrl, Signal
from PySide6.QtGui import QDesktopServices, QFont, QImage, QPixmap, QTextCursor
from PySide6.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QProgressBar,
    QPushButton,
    QSlider,
    QSplitter,
    QVBoxLayout,
    QWidget,
)

from sam3_map_repair_core import (
    DEFAULT_CLASS_PROMPTS,
    MIN_MAPPING_THRESHOLD,
    SEMANTIC_CLASS_REGISTRY,
    CameraFrame,
    find_map_base,
    inspect_bag,
    read_camera_frame,
)


APP_DIR = Path(__file__).resolve().parent
WORKSPACE = APP_DIR.parents[1]
SERVER_DIRECTORY = Path.home() / "sam3_zed_server"
SERVER_URL = "http://localhost:8080"
RUNNER = APP_DIR / "sam3_partial_repair_runner.py"
DEFAULT_BAG = Path.home() / "rosbag2_data/offline_real_20260825_114739_recovered"
DEFAULT_MAP = (
    WORKSPACE
    / "maps_waypoints/maps/rtabmap_semantic_offline_real_20260825_114739_recovered_slam_base_1F-0825-2"
)


def http_json(endpoint: str, payload: dict | None = None, timeout: float = 5.0) -> dict:
    body = None if payload is None else json.dumps(payload).encode("utf-8")
    request = urllib.request.Request(
        SERVER_URL + endpoint,
        data=body,
        headers={"Content-Type": "application/json"},
        method="GET" if payload is None else "POST",
    )
    with urllib.request.urlopen(request, timeout=timeout) as response:
        return json.loads(response.read().decode("utf-8"))


def ensure_preview_server(cancelled=lambda: False) -> None:
    """Start or restart only when the running server lacks the repair API."""
    if cancelled():
        raise InterruptedError
    try:
        http_json("/class_registry", timeout=1.5)
        return
    except (OSError, ValueError, urllib.error.URLError):
        pass
    exists = subprocess.run(
        ["docker", "container", "inspect", "sam3_zed_container"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    ).returncode == 0
    if exists:
        command = ["docker", "restart", "sam3_zed_container"]
        cwd = None
    else:
        command = ["docker", "compose", "up", "-d", "sam3-zed-merged"]
        cwd = SERVER_DIRECTORY
    subprocess.run(command, cwd=cwd, check=True)
    for _ in range(120):
        if cancelled():
            raise InterruptedError
        try:
            http_json("/class_registry", timeout=1.0)
            return
        except (OSError, ValueError, urllib.error.URLError):
            time.sleep(1)
    raise RuntimeError("SAM3サーバーが120秒以内に起動しませんでした。")


def upload_preview(
    frame: CameraFrame,
    prompt: str,
    threshold: float,
    semantic_class: str | None = None,
    cancelled=lambda: False,
) -> tuple[bytes, dict]:
    ensure_preview_server(cancelled)
    if cancelled():
        raise InterruptedError
    before = http_json("/debug_state")
    before_version = int(before.get("sam3_inference_frame_version", -1))
    before_pc_version = int(before.get("pc_version", -1))
    settings = []
    if semantic_class in SEMANTIC_CLASS_REGISTRY:
        settings.append(
            (
                "/class_registry",
                {"classes": {prompt: SEMANTIC_CLASS_REGISTRY[semantic_class]}},
            )
        )
    settings.extend((
        ("/prompt", {"prompt": prompt}),
        ("/threshold", {"threshold": threshold}),
        ("/class_thresholds", {"classes": {prompt: threshold}}),
        ("/sam3_resolution", {"resolution": 512}),
        ("/color_mode", {"mode": "semantic"}),
        ("/source_mode", {"mode": "network"}),
    ))
    for endpoint, payload in settings:
        if cancelled():
            raise InterruptedError
        response = http_json(endpoint, payload)
        if not response.get("ok", False):
            raise RuntimeError(f"設定失敗: {endpoint}: {response}")

    headers = {
        "Content-Type": "image/jpeg",
        "X-Unity-Sim-Time": str(frame.stamp_seconds),
        "X-Camera-Fx": str(frame.camera_params["fx"]),
        "X-Camera-Fy": str(frame.camera_params["fy"]),
        "X-Camera-Cx": str(frame.camera_params["cx"]),
        "X-Camera-Cy": str(frame.camera_params["cy"]),
        "X-Camera-Baseline": str(frame.camera_params["baseline"]),
    }
    request = urllib.request.Request(
        SERVER_URL + "/upload_frame",
        data=frame.jpeg,
        headers=headers,
        method="POST",
    )
    with urllib.request.urlopen(request, timeout=5.0) as response:
        result = json.loads(response.read().decode("utf-8"))
    if not result.get("ok", False):
        raise RuntimeError(f"フレーム送信失敗: {result}")

    state = {}
    for _ in range(180):
        if cancelled():
            raise InterruptedError
        state = http_json("/debug_state", timeout=2.0)
        if (
            int(state.get("sam3_inference_frame_version", -1)) > before_version
            and int(state.get("pc_version", -1)) > before_pc_version
        ):
            try:
                with urllib.request.urlopen(
                    SERVER_URL + "/snapshot.jpg", timeout=3.0
                ) as response:
                    return response.read(), state
            except urllib.error.HTTPError as error:
                if error.code != 404:
                    raise
        time.sleep(0.25)
    raise RuntimeError("SAM3プレビューが45秒以内に完了しませんでした。")


class PreviewThread(QThread):
    completed = Signal(bytes, bytes, object, float)
    failed = Signal(str)

    def __init__(
        self,
        bag: str,
        offset: float,
        prompt: str,
        threshold: float,
        semantic_class: str | None = None,
    ):
        super().__init__()
        self.bag = bag
        self.offset = offset
        self.prompt = prompt
        self.threshold = threshold
        self.semantic_class = semantic_class

    def run(self):
        try:
            frame = read_camera_frame(self.bag, self.offset)
            if self.isInterruptionRequested():
                return
            overlay, state = upload_preview(
                frame,
                self.prompt,
                self.threshold,
                self.semantic_class,
                self.isInterruptionRequested,
            )
            if self.isInterruptionRequested():
                return
            self.completed.emit(
                frame.jpeg, overlay, state, frame.bag_offset_seconds
            )
        except InterruptedError:
            return
        except Exception as error:
            self.failed.emit(str(error))


class FrameLoadThread(QThread):
    completed = Signal(bytes, float, str)
    failed = Signal(str)

    def __init__(self, bag: str, offset: float, position_name: str):
        super().__init__()
        self.bag = bag
        self.offset = offset
        self.position_name = position_name

    def run(self):
        try:
            frame = read_camera_frame(self.bag, self.offset)
            if self.isInterruptionRequested():
                return
            self.completed.emit(
                frame.jpeg, frame.bag_offset_seconds, self.position_name
            )
        except Exception as error:
            self.failed.emit(str(error))


class SegmentPreviewThread(QThread):
    frame_ready = Signal(int, int, str, str, object, float)
    generation_completed = Signal(int)
    generation_cancelled = Signal(int)
    failed = Signal(str)

    def __init__(
        self,
        bag: str,
        offsets: list[float],
        prompt: str,
        threshold: float,
        cache_directory: str,
        semantic_class: str | None = None,
    ):
        super().__init__()
        self.bag = bag
        self.offsets = offsets
        self.prompt = prompt
        self.threshold = threshold
        self.cache_directory = Path(cache_directory)
        self.semantic_class = semantic_class

    def run(self):
        completed = 0
        try:
            for index, offset in enumerate(self.offsets):
                if self.isInterruptionRequested():
                    self.generation_cancelled.emit(completed)
                    return
                frame = read_camera_frame(self.bag, offset)
                overlay, state = upload_preview(
                    frame,
                    self.prompt,
                    self.threshold,
                    self.semantic_class,
                    self.isInterruptionRequested,
                )
                if self.isInterruptionRequested():
                    self.generation_cancelled.emit(completed)
                    return
                raw_path = self.cache_directory / f"{index:06d}_raw.jpg"
                overlay_path = self.cache_directory / f"{index:06d}_sam3.jpg"
                raw_path.write_bytes(frame.jpeg)
                overlay_path.write_bytes(overlay)
                completed += 1
                self.frame_ready.emit(
                    index,
                    len(self.offsets),
                    str(raw_path),
                    str(overlay_path),
                    state,
                    frame.bag_offset_seconds,
                )
            self.generation_completed.emit(completed)
        except InterruptedError:
            self.generation_cancelled.emit(completed)
        except Exception as error:
            self.failed.emit(str(error))


class ImageLabel(QLabel):
    def __init__(self, placeholder: str):
        super().__init__(placeholder)
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setMinimumSize(360, 230)
        self.setStyleSheet("background:#202124; color:#c8c8c8; border:1px solid #555;")
        self.setScaledContents(False)
        self._pixmap = None

    def set_image(self, data: bytes):
        image = QImage.fromData(data)
        if image.isNull():
            self.setText("画像を読み込めません")
            return
        self._pixmap = QPixmap.fromImage(image)
        self._refresh()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._refresh()

    def _refresh(self):
        if self._pixmap:
            self.setPixmap(
                self._pixmap.scaled(
                    self.size(),
                    Qt.AspectRatioMode.KeepAspectRatio,
                    Qt.TransformationMode.SmoothTransformation,
                )
            )


class SAM3MapRepairWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("SAM3 部分地図補正")
        self.resize(1180, 860)
        self.summary = None
        self.preview_thread = None
        self.frame_thread = None
        self.segment_preview_thread = None
        self.active_threads = set()
        self.close_after_threads = False
        self.segment_preview_frames = []
        self.segment_preview_cache = None
        self.segment_preview_class = ""
        self.playback_index = 0
        self.process = None
        self.output_directory = None
        self._stdout_buffer = ""
        self._syncing_time_controls = False
        self.class_threshold_values = {
            "tactile paving": 0.40,
            "grass": 0.50,
            "roadway": 0.50,
            "sidewalk": 0.50,
        }
        self.class_prompt_values = dict(DEFAULT_CLASS_PROMPTS)
        self.class_checks = {}
        self.custom_preview_classes = set()
        self.pending_slider_preview = None
        self.last_slider_position = ("開始", 0.0)
        self.sam_slider_timer = QTimer(self)
        self.sam_slider_timer.setSingleShot(True)
        self.sam_slider_timer.setInterval(700)
        self.sam_slider_timer.timeout.connect(self.run_scheduled_sam_preview)
        self.playback_timer = QTimer(self)
        self.playback_timer.setInterval(250)
        self.playback_timer.timeout.connect(self.playback_tick)
        self._build_ui()
        self.load_custom_preview_classes()
        self.load_class_prompts()
        self.preview_class_changed(self.class_combo.currentText())
        if DEFAULT_BAG.is_dir():
            self.bag_edit.setText(str(DEFAULT_BAG))
        if DEFAULT_MAP.is_dir():
            self.map_edit.setText(str(DEFAULT_MAP))
        if self.bag_edit.text():
            self.inspect_inputs()

    def _build_ui(self):
        central = QWidget()
        root = QVBoxLayout(central)
        root.setContentsMargins(16, 16, 16, 16)
        root.setSpacing(10)
        title = QLabel("SAM3 部分地図補正")
        title.setFont(QFont("", 18, QFont.Weight.Bold))
        root.addWidget(title)
        description = QLabel(
            "rosbagの指定区間だけを対象クラスで再推論し、既存地図へ追加します。"
            "元のrosbagと地図は上書きしません。"
        )
        description.setWordWrap(True)
        root.addWidget(description)

        inputs = QGroupBox("1. 入力")
        grid = QGridLayout(inputs)
        self.bag_edit = QLineEdit()
        self.map_edit = QLineEdit()
        bag_button = QPushButton("Rosbag選択")
        map_button = QPushButton("既存地図選択")
        inspect_button = QPushButton("検証")
        bag_button.clicked.connect(self.select_bag)
        map_button.clicked.connect(self.select_map)
        inspect_button.clicked.connect(self.inspect_inputs)
        grid.addWidget(QLabel("Rosbag"), 0, 0)
        grid.addWidget(self.bag_edit, 0, 1)
        grid.addWidget(bag_button, 0, 2)
        grid.addWidget(QLabel("補正元地図"), 1, 0)
        grid.addWidget(self.map_edit, 1, 1)
        grid.addWidget(map_button, 1, 2)
        grid.addWidget(inspect_button, 2, 2)
        self.input_status = QLabel("入力を選択してください。")
        self.input_status.setWordWrap(True)
        grid.addWidget(self.input_status, 2, 0, 1, 2)
        grid.setColumnStretch(1, 1)
        root.addWidget(inputs)

        controls = QGroupBox("2. 区間・対象クラス・信頼度")
        controls_layout = QGridLayout(controls)
        self.start_spin = QDoubleSpinBox()
        self.end_spin = QDoubleSpinBox()
        for spin in (self.start_spin, self.end_spin):
            spin.setDecimals(2)
            spin.setSuffix(" s")
            spin.setSingleStep(0.1)
        self.start_slider = QSlider(Qt.Orientation.Horizontal)
        self.end_slider = QSlider(Qt.Orientation.Horizontal)
        for slider in (self.start_slider, self.end_slider):
            slider.setRange(0, 1)
            slider.setSingleStep(1)
            slider.setPageStep(50)
            slider.setTracking(True)
        self.start_slider.valueChanged.connect(self.start_slider_changed)
        self.end_slider.valueChanged.connect(self.end_slider_changed)
        self.start_spin.valueChanged.connect(self.start_spin_changed)
        self.end_spin.valueChanged.connect(self.end_spin_changed)
        self.start_frame_button = QPushButton("開始画像を見る")
        self.end_frame_button = QPushButton("終了画像を見る")
        self.start_frame_button.clicked.connect(
            lambda: self.load_position_frame("開始", self.start_spin.value())
        )
        self.end_frame_button.clicked.connect(
            lambda: self.load_position_frame("終了", self.end_spin.value())
        )
        self.range_label = QLabel("選択区間: -")
        self.class_combo = QComboBox()
        self.class_combo.addItems(
            ["tactile paving", "grass", "roadway", "sidewalk"]
        )
        self.class_combo.currentTextChanged.connect(self.preview_class_changed)
        class_selection_box = QHBoxLayout()
        for class_name in self.class_threshold_values:
            checkbox = QCheckBox(class_name)
            checkbox.setChecked(class_name == "tactile paving")
            self.class_checks[class_name] = checkbox
            class_selection_box.addWidget(checkbox)
        class_selection_box.addStretch(1)
        self.threshold_spin = QDoubleSpinBox()
        self.threshold_spin.setRange(0.0, 1.0)
        self.threshold_spin.setSingleStep(0.05)
        self.threshold_spin.setDecimals(2)
        self.threshold_spin.setValue(0.40)
        self.threshold_slider = QSlider(Qt.Orientation.Horizontal)
        self.threshold_slider.setRange(0, 100)
        self.threshold_slider.setValue(40)
        self.threshold_slider.valueChanged.connect(
            lambda value: self.threshold_spin.setValue(value / 100.0)
        )
        self.threshold_spin.valueChanged.connect(self.preview_threshold_changed)
        self.rate_combo = QComboBox()
        self.rate_combo.addItems(["0.25", "0.5", "1.0"])
        controls_layout.addWidget(QLabel("開始位置"), 0, 0)
        controls_layout.addWidget(self.start_slider, 0, 1, 1, 2)
        controls_layout.addWidget(self.start_spin, 0, 3)
        controls_layout.addWidget(self.start_frame_button, 0, 4)
        controls_layout.addWidget(QLabel("終了位置"), 1, 0)
        controls_layout.addWidget(self.end_slider, 1, 1, 1, 2)
        controls_layout.addWidget(self.end_spin, 1, 3)
        controls_layout.addWidget(self.end_frame_button, 1, 4)
        controls_layout.addWidget(self.range_label, 2, 1, 1, 4)
        controls_layout.addWidget(QLabel("再推論クラス"), 3, 0)
        controls_layout.addLayout(class_selection_box, 3, 1, 1, 4)
        controls_layout.addWidget(QLabel("プレビュー・閾値編集"), 4, 0)
        controls_layout.addWidget(self.class_combo, 4, 1)
        controls_layout.addWidget(QLabel("選択クラスの閾値"), 4, 2)
        threshold_box = QHBoxLayout()
        threshold_box.addWidget(self.threshold_slider, 1)
        threshold_box.addWidget(self.threshold_spin)
        controls_layout.addLayout(threshold_box, 4, 3, 1, 2)
        self.prompt_edit = QLineEdit()
        self.prompt_edit.setPlaceholderText("SAM3へ渡す英語の推論文")
        self.prompt_edit.returnPressed.connect(self.apply_class_prompt)
        self.apply_prompt_button = QPushButton("推論文を適用")
        self.apply_prompt_button.clicked.connect(self.apply_class_prompt)
        controls_layout.addWidget(QLabel("現在クラスの推論文"), 5, 0)
        controls_layout.addWidget(self.prompt_edit, 5, 1, 1, 3)
        controls_layout.addWidget(self.apply_prompt_button, 5, 4)
        self.custom_class_edit = QLineEdit()
        self.custom_class_edit.setPlaceholderText("例: curb, crosswalk, person")
        self.custom_class_edit.returnPressed.connect(self.add_preview_class)
        self.add_class_button = QPushButton("推論名を追加")
        self.add_class_button.clicked.connect(self.add_preview_class)
        self.remove_class_button = QPushButton("追加名を削除")
        self.remove_class_button.clicked.connect(self.remove_preview_class)
        controls_layout.addWidget(QLabel("プレビュー名追加"), 6, 0)
        controls_layout.addWidget(self.custom_class_edit, 6, 1, 1, 2)
        controls_layout.addWidget(self.add_class_button, 6, 3)
        controls_layout.addWidget(self.remove_class_button, 6, 4)
        controls_layout.addWidget(QLabel("再生速度"), 7, 0)
        controls_layout.addWidget(self.rate_combo, 7, 1)
        note = QLabel(
            "追加した推論名は1フレーム・区間プレビュー用です。"
            "地図補正対象は上段の4クラス、地図生成の最低閾値は0.20です。"
            "スコアは正解確率ではありません。"
        )
        note.setWordWrap(True)
        controls_layout.addWidget(note, 7, 2, 1, 3)
        root.addWidget(controls)

        preview_group = QGroupBox("3. SAM3プレビュー")
        preview_layout = QVBoxLayout(preview_group)
        preview_button_row = QHBoxLayout()
        self.preview_button = QPushButton("開始位置をSAM3プレビュー")
        self.preview_button.clicked.connect(lambda: self.start_preview())
        self.auto_sam_checkbox = QCheckBox("スライダー停止後にSAM3も自動実行（重い）")
        self.auto_sam_checkbox.setChecked(False)
        self.score_label = QLabel("採用マスクスコア: -")
        self.score_label.setTextInteractionFlags(
            Qt.TextInteractionFlag.TextSelectableByMouse
        )
        preview_button_row.addWidget(self.preview_button)
        preview_button_row.addWidget(self.auto_sam_checkbox)
        preview_button_row.addWidget(self.score_label, 1)
        preview_layout.addLayout(preview_button_row)

        segment_button_row = QHBoxLayout()
        self.segment_preview_button = QPushButton("開始～終了をSAM3プレビュー作成")
        self.segment_preview_button.clicked.connect(self.start_segment_preview)
        self.segment_cancel_button = QPushButton("作成中止")
        self.segment_cancel_button.setEnabled(False)
        self.segment_cancel_button.clicked.connect(self.cancel_segment_preview)
        self.segment_interval_combo = QComboBox()
        self.segment_interval_combo.addItems(["0.5", "1.0", "2.0", "5.0"])
        self.segment_interval_combo.setCurrentText("1.0")
        self.playback_speed_combo = QComboBox()
        self.playback_speed_combo.addItems(["1", "2", "4", "8"])
        self.playback_speed_combo.setCurrentText("4")
        self.playback_speed_combo.currentTextChanged.connect(
            self.playback_speed_changed
        )
        segment_button_row.addWidget(self.segment_preview_button)
        segment_button_row.addWidget(self.segment_cancel_button)
        segment_button_row.addWidget(QLabel("推論間隔"))
        segment_button_row.addWidget(self.segment_interval_combo)
        segment_button_row.addWidget(QLabel("秒 / 再生"))
        segment_button_row.addWidget(self.playback_speed_combo)
        segment_button_row.addWidget(QLabel("fps"))
        segment_button_row.addStretch(1)
        preview_layout.addLayout(segment_button_row)

        playback_row = QHBoxLayout()
        self.playback_button = QPushButton("▶ 再生")
        self.playback_button.setEnabled(False)
        self.playback_button.clicked.connect(self.toggle_playback)
        self.playback_slider = QSlider(Qt.Orientation.Horizontal)
        self.playback_slider.setRange(0, 0)
        self.playback_slider.setEnabled(False)
        self.playback_slider.valueChanged.connect(self.show_segment_frame)
        self.playback_label = QLabel("区間プレビュー未作成")
        playback_row.addWidget(self.playback_button)
        playback_row.addWidget(self.playback_slider, 1)
        playback_row.addWidget(self.playback_label)
        preview_layout.addLayout(playback_row)
        self.segment_progress = QProgressBar()
        self.segment_progress.setRange(0, 1)
        self.segment_progress.setValue(0)
        preview_layout.addWidget(self.segment_progress)

        image_splitter = QSplitter(Qt.Orientation.Horizontal)
        self.raw_image = ImageLabel("Rosbagカメラ画像")
        self.mask_image = ImageLabel("SAM3マスク結果")
        image_splitter.addWidget(self.raw_image)
        image_splitter.addWidget(self.mask_image)
        image_splitter.setSizes([550, 550])
        preview_layout.addWidget(image_splitter)
        root.addWidget(preview_group, 1)

        actions = QGroupBox("4. 部分再推論・地図マージ")
        actions_layout = QVBoxLayout(actions)
        button_row = QHBoxLayout()
        self.run_button = QPushButton("この区間を再推論して新しい地図を作成")
        self.run_button.clicked.connect(self.start_repair)
        self.stop_button = QPushButton("中止")
        self.stop_button.setEnabled(False)
        self.stop_button.clicked.connect(self.stop_repair)
        self.open_button = QPushButton("出力地図を開く")
        self.open_button.setEnabled(False)
        self.open_button.clicked.connect(self.open_output)
        button_row.addWidget(self.run_button)
        button_row.addWidget(self.stop_button)
        button_row.addStretch(1)
        button_row.addWidget(self.open_button)
        actions_layout.addLayout(button_row)
        self.progress = QProgressBar()
        self.progress.setRange(0, 1)
        self.progress.setValue(0)
        actions_layout.addWidget(self.progress)
        self.log = QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(4000)
        self.log.setPlaceholderText("実行ログ")
        actions_layout.addWidget(self.log)
        root.addWidget(actions, 1)
        self.setCentralWidget(central)

    def select_bag(self):
        selected = QFileDialog.getExistingDirectory(
            self,
            "Rosbagディレクトリを選択",
            self.bag_edit.text() or str(Path.home() / "rosbag2_data"),
        )
        if selected:
            self.bag_edit.setText(selected)
            self.inspect_inputs()

    def select_map(self):
        selected = QFileDialog.getExistingDirectory(
            self,
            "補正元の地図ディレクトリを選択",
            self.map_edit.text() or str(WORKSPACE / "maps_waypoints/maps"),
        )
        if selected:
            self.map_edit.setText(selected)
            self.inspect_inputs()

    @staticmethod
    def format_time(seconds: float) -> str:
        minutes = int(seconds) // 60
        remainder = seconds - minutes * 60
        return f"{minutes:02d}:{remainder:04.1f}"

    def preview_class_changed(self, class_name: str):
        if not class_name or not hasattr(self, "threshold_spin"):
            return
        self._syncing_time_controls = True
        try:
            threshold = self.class_threshold_values.get(class_name, 0.5)
            self.threshold_spin.setValue(threshold)
            self.threshold_slider.setValue(round(threshold * 100))
            if hasattr(self, "prompt_edit"):
                self.prompt_edit.setText(
                    self.class_prompt_values.get(class_name, class_name)
                )
        finally:
            self._syncing_time_controls = False

    def preview_threshold_changed(self, value: float):
        self.threshold_slider.setValue(round(value * 100))
        if not self._syncing_time_controls:
            self.class_threshold_values[self.class_combo.currentText()] = float(value)

    def selected_class_thresholds(self) -> dict[str, float]:
        return {
            name: self.class_threshold_values[name]
            for name, checkbox in self.class_checks.items()
            if checkbox.isChecked()
        }

    def load_custom_preview_classes(self):
        stored = QSettings("Sirius", "SAM3MapRepair").value(
            "custom_preview_classes", []
        )
        if isinstance(stored, str):
            stored = [stored]
        for class_name in stored or []:
            self.register_preview_class(str(class_name), persist=False)

    def load_class_prompts(self):
        raw = QSettings("Sirius", "SAM3MapRepair").value("class_prompts", "")
        if raw:
            try:
                stored = json.loads(str(raw))
            except (TypeError, ValueError, json.JSONDecodeError):
                stored = {}
            if isinstance(stored, dict):
                for class_name, prompt in stored.items():
                    if class_name in self.class_prompt_values:
                        normalized = " ".join(str(prompt).strip().split())
                        if normalized and "," not in normalized:
                            self.class_prompt_values[class_name] = normalized

    def save_class_prompts(self):
        QSettings("Sirius", "SAM3MapRepair").setValue(
            "class_prompts",
            json.dumps(self.class_prompt_values, ensure_ascii=False),
        )

    def apply_class_prompt(self):
        class_name = self.class_combo.currentText()
        prompt = " ".join(self.prompt_edit.text().strip().split())
        if not prompt or "," in prompt or len(prompt) > 120:
            QMessageBox.warning(
                self,
                "SAM3推論文",
                "1～120文字で入力してください。カンマは使用できません。",
            )
            return
        self.class_prompt_values[class_name] = prompt
        self.save_class_prompts()
        self.score_label.setText(
            f"{class_name} のSAM3推論文を「{prompt}」に設定しました。"
        )

    def save_custom_preview_classes(self):
        QSettings("Sirius", "SAM3MapRepair").setValue(
            "custom_preview_classes", sorted(self.custom_preview_classes)
        )

    def register_preview_class(self, class_name: str, persist: bool = True) -> bool:
        class_name = " ".join(class_name.strip().split())
        if not class_name or "," in class_name or len(class_name) > 80:
            return False
        if class_name not in self.class_threshold_values:
            self.class_threshold_values[class_name] = 0.50
            self.class_prompt_values[class_name] = class_name
            self.class_combo.addItem(class_name)
            self.custom_preview_classes.add(class_name)
        elif class_name not in self.class_checks:
            self.custom_preview_classes.add(class_name)
        self.class_combo.setCurrentText(class_name)
        if persist:
            self.save_custom_preview_classes()
        return True

    def add_preview_class(self):
        class_name = self.custom_class_edit.text()
        if not self.register_preview_class(class_name):
            QMessageBox.warning(
                self,
                "推論名",
                "1～80文字で入力してください。カンマは使用できません。",
            )
            return
        self.custom_class_edit.clear()
        self.score_label.setText(
            f"推論名「{self.class_combo.currentText()}」を追加しました（プレビュー用）。"
        )

    def remove_preview_class(self):
        class_name = self.class_combo.currentText()
        if class_name not in self.custom_preview_classes:
            QMessageBox.information(
                self, "推論名", "標準の地図クラスは削除できません。"
            )
            return
        if (
            (self.preview_thread and self.preview_thread.isRunning())
            or (
                self.segment_preview_thread
                and self.segment_preview_thread.isRunning()
            )
        ):
            QMessageBox.warning(self, "SAM3使用中", "プレビュー完了後に削除してください。")
            return
        index = self.class_combo.findText(class_name)
        if index >= 0:
            self.class_combo.removeItem(index)
        self.custom_preview_classes.discard(class_name)
        self.class_threshold_values.pop(class_name, None)
        self.class_prompt_values.pop(class_name, None)
        self.save_custom_preview_classes()
        self.save_class_prompts()

    def start_slider_changed(self, raw_value: int):
        if self._syncing_time_controls:
            return
        seconds = raw_value / 10.0
        if self.summary:
            seconds = min(seconds, max(0.0, self.summary.duration_seconds - 0.1))
        self._syncing_time_controls = True
        try:
            self.start_spin.setValue(seconds)
            corrected = round(seconds * 10)
            if corrected != raw_value:
                self.start_slider.setValue(corrected)
            if self.end_spin.value() <= seconds:
                new_end = min(
                    self.summary.duration_seconds if self.summary else seconds + 0.1,
                    seconds + 0.1,
                )
                self.end_spin.setValue(new_end)
                self.end_slider.setValue(round(new_end * 10))
        finally:
            self._syncing_time_controls = False
        self.update_range_label()
        self.schedule_slider_preview("開始", seconds)

    def end_slider_changed(self, raw_value: int):
        if self._syncing_time_controls:
            return
        seconds = max(raw_value / 10.0, 0.1)
        if self.summary:
            seconds = min(seconds, self.summary.duration_seconds)
        self._syncing_time_controls = True
        try:
            self.end_spin.setValue(seconds)
            corrected = round(seconds * 10)
            if corrected != raw_value:
                self.end_slider.setValue(corrected)
            if self.start_spin.value() >= seconds:
                new_start = max(0.0, seconds - 0.1)
                self.start_spin.setValue(new_start)
                self.start_slider.setValue(round(new_start * 10))
        finally:
            self._syncing_time_controls = False
        self.update_range_label()
        self.schedule_slider_preview("終了", seconds)

    def start_spin_changed(self, seconds: float):
        if self._syncing_time_controls:
            return
        self._syncing_time_controls = True
        try:
            self.start_slider.setValue(round(seconds * 10))
        finally:
            self._syncing_time_controls = False
        self.update_range_label()
        self.schedule_slider_preview("開始", seconds)

    def end_spin_changed(self, seconds: float):
        if self._syncing_time_controls:
            return
        self._syncing_time_controls = True
        try:
            self.end_slider.setValue(round(seconds * 10))
        finally:
            self._syncing_time_controls = False
        self.update_range_label()
        self.schedule_slider_preview("終了", seconds)

    def update_range_label(self):
        start = self.start_spin.value()
        end = self.end_spin.value()
        self.range_label.setText(
            f"選択区間: {self.format_time(start)} ～ {self.format_time(end)} "
            f"（{max(0.0, end - start):.1f}秒）"
        )

    def schedule_slider_preview(self, position_name: str, seconds: float):
        if self._syncing_time_controls or not self.summary:
            return
        self.last_slider_position = (position_name, seconds)
        self.pending_slider_preview = (position_name, seconds)
        if not self.frame_thread or not self.frame_thread.isRunning():
            self.run_scheduled_slider_preview()
        if self.auto_sam_checkbox.isChecked():
            self.sam_slider_timer.start()
        else:
            self.sam_slider_timer.stop()

    def run_scheduled_slider_preview(self):
        if not self.pending_slider_preview:
            return
        position_name, seconds = self.pending_slider_preview
        self.pending_slider_preview = None
        self.load_position_frame(position_name, seconds)

    def run_scheduled_sam_preview(self):
        if not self.auto_sam_checkbox.isChecked():
            return
        position_name, seconds = self.last_slider_position
        if self.preview_thread and self.preview_thread.isRunning():
            self.sam_slider_timer.start()
            return
        self.start_preview(seconds, position_name)

    def load_position_frame(self, position_name: str, seconds: float):
        if not self.summary and not self.inspect_inputs():
            return
        request = (position_name, float(seconds))
        if self.frame_thread and self.frame_thread.isRunning():
            self.pending_slider_preview = request
            return
        self.score_label.setText(
            f"{position_name} {self.format_time(seconds)} のカメラ画像を読込中…"
        )
        self.frame_thread = FrameLoadThread(
            self.bag_edit.text().strip(), seconds, position_name
        )
        thread = self.frame_thread
        self.active_threads.add(thread)
        self.frame_thread.completed.connect(self.position_frame_completed)
        self.frame_thread.failed.connect(self.position_frame_failed)
        self.frame_thread.finished.connect(
            lambda current=thread: self.position_frame_finished(current)
        )
        self.frame_thread.start()

    def position_frame_completed(self, raw: bytes, offset: float, position_name: str):
        self.raw_image.set_image(raw)
        self.score_label.setText(
            f"{position_name}位置 {self.format_time(offset)} / カメラ画像のみ（SAM3未実行）"
        )

    def position_frame_failed(self, message: str):
        self.score_label.setText(f"✗ 位置画像を取得できません: {message}")

    def position_frame_finished(self, thread: FrameLoadThread):
        thread.wait()
        if self.frame_thread is thread:
            self.frame_thread = None
        self.active_threads.discard(thread)
        thread.deleteLater()
        if self.pending_slider_preview and not self.close_after_threads:
            QTimer.singleShot(0, self.run_scheduled_slider_preview)
        self.finish_deferred_close()

    def inspect_inputs(self) -> bool:
        try:
            self.summary = inspect_bag(self.bag_edit.text().strip())
            if self.summary.missing_topics:
                raise ValueError(
                    "必須トピック不足: "
                    + ", ".join(sorted(self.summary.missing_topics))
                )
            map_base = find_map_base(self.map_edit.text().strip())
            self._syncing_time_controls = True
            try:
                self.start_spin.setRange(0, self.summary.duration_seconds)
                self.end_spin.setRange(0, self.summary.duration_seconds)
                if (
                    self.end_spin.value() <= 0
                    or self.end_spin.value() > self.summary.duration_seconds
                ):
                    self.end_spin.setValue(min(30.0, self.summary.duration_seconds))
                slider_max = max(1, round(self.summary.duration_seconds * 10))
                self.start_slider.setRange(0, slider_max)
                self.end_slider.setRange(0, slider_max)
                self.start_slider.setValue(round(self.start_spin.value() * 10))
                self.end_slider.setValue(round(self.end_spin.value() * 10))
            finally:
                self._syncing_time_controls = False
            self.update_range_label()
            self.input_status.setText(
                f"✓ bag長 {self.summary.duration_seconds:.2f}s / 地図 {map_base.name} / "
                "TF・カメラ情報あり"
            )
            self.run_button.setEnabled(True)
            return True
        except Exception as error:
            self.summary = None
            self.input_status.setText(f"✗ {error}")
            self.run_button.setEnabled(False)
            return False

    def start_segment_preview(self):
        if not self.inspect_inputs():
            return
        if self.end_spin.value() <= self.start_spin.value():
            QMessageBox.warning(self, "時間範囲", "終了時刻を開始時刻より後にしてください。")
            return
        if self.preview_thread and self.preview_thread.isRunning():
            QMessageBox.warning(
                self, "SAM3使用中", "1フレームのSAM3プレビュー完了後に実行してください。"
            )
            return
        if self.segment_preview_thread and self.segment_preview_thread.isRunning():
            return
        if self.process and self.process.state() != QProcess.ProcessState.NotRunning:
            QMessageBox.warning(self, "地図作成中", "地図作成の完了後に実行してください。")
            return

        start = self.start_spin.value()
        end = self.end_spin.value()
        interval = float(self.segment_interval_combo.currentText())
        offsets = []
        offset = start
        while offset <= end + 1e-6:
            offsets.append(min(offset, end))
            offset += interval
        if not offsets or end - offsets[-1] > 1e-3:
            offsets.append(end)
        target_class = self.class_combo.currentText()
        prompt = self.class_prompt_values.get(target_class, target_class)
        threshold = self.class_threshold_values[target_class]
        answer = QMessageBox.question(
            self,
            "区間SAM3プレビュー",
            f"{self.format_time(start)} ～ {self.format_time(end)} を"
            f" {interval:.1f}秒間隔で処理します。\n"
            f"意味クラス: {target_class}\n"
            f"SAM3推論文: {prompt} / 閾値 {threshold:.2f}\n"
            f"推論フレーム数: {len(offsets)}\n\n"
            "SAM3推論のため、作成には時間がかかります。開始しますか？",
        )
        if answer != QMessageBox.StandardButton.Yes:
            return

        self.playback_timer.stop()
        self.clear_segment_preview_cache()
        self.segment_preview_cache = tempfile.TemporaryDirectory(
            prefix="sam3_map_repair_preview_"
        )
        self.segment_preview_frames = []
        self.segment_preview_class = target_class
        self.segment_preview_prompt = prompt
        self.playback_index = 0
        self.playback_slider.setRange(0, max(0, len(offsets) - 1))
        self.playback_slider.setValue(0)
        self.playback_slider.setEnabled(False)
        self.playback_button.setEnabled(False)
        self.playback_button.setText("▶ 再生")
        self.segment_progress.setRange(0, len(offsets))
        self.segment_progress.setValue(0)
        self.playback_label.setText(f"0 / {len(offsets)} フレーム")
        self.segment_preview_button.setEnabled(False)
        self.segment_cancel_button.setEnabled(True)
        self.preview_button.setEnabled(False)
        self.sam_slider_timer.stop()

        self.segment_preview_thread = SegmentPreviewThread(
            self.bag_edit.text().strip(),
            offsets,
            prompt,
            threshold,
            self.segment_preview_cache.name,
            target_class if target_class in SEMANTIC_CLASS_REGISTRY else None,
        )
        thread = self.segment_preview_thread
        self.active_threads.add(thread)
        thread.frame_ready.connect(self.segment_frame_ready)
        thread.generation_completed.connect(self.segment_generation_completed)
        thread.generation_cancelled.connect(self.segment_generation_cancelled)
        thread.failed.connect(self.segment_generation_failed)
        thread.finished.connect(
            lambda current=thread: self.segment_preview_finished(current)
        )
        thread.start()

    def cancel_segment_preview(self):
        if self.segment_preview_thread and self.segment_preview_thread.isRunning():
            self.segment_preview_thread.requestInterruption()
            self.segment_cancel_button.setEnabled(False)
            self.playback_label.setText("SAM3区間プレビューを安全に中止中…")

    def segment_frame_ready(
        self,
        index: int,
        total: int,
        raw_path: str,
        overlay_path: str,
        state: object,
        offset: float,
    ):
        scores = state.get("class_scores", {}).get(self.segment_preview_prompt, [])
        frame_data = {
            "raw_path": raw_path,
            "overlay_path": overlay_path,
            "offset": float(offset),
            "scores": [float(score) for score in scores],
        }
        while len(self.segment_preview_frames) <= index:
            self.segment_preview_frames.append(None)
        self.segment_preview_frames[index] = frame_data
        self.segment_progress.setValue(index + 1)
        self.playback_slider.setEnabled(True)
        if self.playback_slider.value() == index:
            self.show_segment_frame(index)
        else:
            self.playback_slider.setValue(index)
        self.playback_label.setText(
            f"推論中 {index + 1} / {total}・{self.format_time(offset)}"
        )

    def segment_generation_completed(self, count: int):
        self.segment_progress.setValue(count)
        self.playback_label.setText(f"✓ {count}フレーム作成完了")
        if count:
            self.playback_button.setEnabled(True)
            self.playback_slider.setValue(0)
            self.start_playback()

    def segment_generation_cancelled(self, count: int):
        self.playback_label.setText(f"中止しました（作成済み {count}フレーム）")
        self.playback_button.setEnabled(count > 0)

    def segment_generation_failed(self, message: str):
        self.playback_label.setText(f"✗ 区間プレビュー失敗: {message}")
        if not self.close_after_threads:
            QMessageBox.warning(self, "区間プレビュー失敗", message)

    def segment_preview_finished(self, thread: SegmentPreviewThread):
        thread.wait()
        if self.segment_preview_thread is thread:
            self.segment_preview_thread = None
        self.active_threads.discard(thread)
        thread.deleteLater()
        self.segment_preview_button.setEnabled(True)
        self.segment_cancel_button.setEnabled(False)
        self.preview_button.setEnabled(True)
        self.playback_button.setEnabled(bool(self.segment_preview_frames))
        self.finish_deferred_close()

    def show_segment_frame(self, index: int):
        if not 0 <= index < len(self.segment_preview_frames):
            return
        frame = self.segment_preview_frames[index]
        if not frame:
            return
        raw_path = Path(frame["raw_path"])
        overlay_path = Path(frame["overlay_path"])
        if not raw_path.is_file() or not overlay_path.is_file():
            return
        self.playback_index = index
        self.raw_image.set_image(raw_path.read_bytes())
        self.mask_image.set_image(overlay_path.read_bytes())
        scores = frame["scores"]
        score_text = (
            ", ".join(f"{score:.3f}" for score in scores)
            if scores
            else "採用なし"
        )
        self.score_label.setText(
            f"区間 {self.format_time(frame['offset'])} / {self.segment_preview_class} "
            f"← '{self.segment_preview_prompt}' / "
            f"採用マスクスコア: {score_text}"
        )
        self.playback_label.setText(
            f"{index + 1} / {len(self.segment_preview_frames)}・"
            f"{self.format_time(frame['offset'])}"
        )

    def playback_speed_changed(self, frames_per_second: str):
        fps = max(1, int(frames_per_second))
        self.playback_timer.setInterval(max(20, round(1000 / fps)))

    def toggle_playback(self):
        if self.playback_timer.isActive():
            self.stop_playback()
        else:
            self.start_playback()

    def start_playback(self):
        if not self.segment_preview_frames:
            return
        if self.playback_slider.value() >= len(self.segment_preview_frames) - 1:
            self.playback_slider.setValue(0)
        self.playback_timer.start()
        self.playback_button.setText("⏸ 一時停止")

    def stop_playback(self):
        self.playback_timer.stop()
        self.playback_button.setText("▶ 再生")

    def playback_tick(self):
        next_index = self.playback_slider.value() + 1
        if next_index >= len(self.segment_preview_frames):
            self.stop_playback()
            return
        self.playback_slider.setValue(next_index)

    def clear_segment_preview_cache(self):
        self.playback_timer.stop()
        self.segment_preview_frames = []
        if self.segment_preview_cache is not None:
            self.segment_preview_cache.cleanup()
            self.segment_preview_cache = None

    def start_preview(
        self, offset_seconds: float | None = None, position_name: str = "開始"
    ):
        if not self.inspect_inputs():
            return
        if self.segment_preview_thread and self.segment_preview_thread.isRunning():
            self.score_label.setText("区間SAM3プレビュー作成中です。")
            return
        if self.preview_thread and self.preview_thread.isRunning():
            self.score_label.setText("SAM3推論中です。完了後に再度操作してください。")
            return
        if offset_seconds is None:
            offset_seconds = self.start_spin.value()
            position_name = "開始"
        target_class = self.class_combo.currentText()
        prompt = self.class_prompt_values.get(target_class, target_class)
        threshold = self.class_threshold_values[target_class]
        self.active_sam_class = target_class
        self.active_sam_prompt = prompt
        self.active_sam_position_name = position_name
        self.preview_button.setEnabled(False)
        self.score_label.setText(
            f"{position_name} {self.format_time(offset_seconds)} / "
            f"{target_class} ← '{prompt}' をSAM3推論中…"
        )
        self.preview_thread = PreviewThread(
            self.bag_edit.text().strip(),
            offset_seconds,
            prompt,
            threshold,
            target_class if target_class in SEMANTIC_CLASS_REGISTRY else None,
        )
        thread = self.preview_thread
        self.active_threads.add(thread)
        self.preview_thread.completed.connect(self.preview_completed)
        self.preview_thread.failed.connect(self.preview_failed)
        self.preview_thread.finished.connect(
            lambda current=thread: self.preview_finished(current)
        )
        self.preview_thread.start()

    def preview_finished(self, thread: PreviewThread):
        thread.wait()
        if self.preview_thread is thread:
            self.preview_thread = None
        self.active_threads.discard(thread)
        thread.deleteLater()
        self.preview_button.setEnabled(True)
        self.finish_deferred_close()

    def finish_deferred_close(self):
        if self.close_after_threads and not any(
            thread.isRunning() for thread in self.active_threads
        ):
            self.close_after_threads = False
            QTimer.singleShot(0, self.close)

    def preview_completed(self, raw: bytes, overlay: bytes, state: object, offset: float):
        self.raw_image.set_image(raw)
        self.mask_image.set_image(overlay)
        target_class = getattr(self, "active_sam_class", self.class_combo.currentText())
        prompt = getattr(
            self,
            "active_sam_prompt",
            self.class_prompt_values.get(target_class, target_class),
        )
        position_name = getattr(self, "active_sam_position_name", "開始")
        scores = state.get("class_scores", {}).get(prompt, [])
        if scores:
            score_text = ", ".join(f"{float(score):.3f}" for score in scores)
        else:
            score_text = "採用なし（閾値未満または未検出）"
        self.score_label.setText(
            f"{position_name}位置 {self.format_time(offset)} / "
            f"{target_class} ← '{prompt}' / "
            f"採用マスクスコア: {score_text}"
        )

    def preview_failed(self, message: str):
        self.score_label.setText(f"✗ {message}")
        QMessageBox.warning(self, "プレビュー失敗", message)

    def start_repair(self):
        if not self.inspect_inputs():
            return
        if (
            (self.preview_thread and self.preview_thread.isRunning())
            or (
                self.segment_preview_thread
                and self.segment_preview_thread.isRunning()
            )
        ):
            QMessageBox.warning(
                self, "SAM3使用中", "SAM3プレビューを完了または中止してから実行してください。"
            )
            return
        if self.end_spin.value() <= self.start_spin.value():
            QMessageBox.warning(self, "時間範囲", "終了時刻を開始時刻より後にしてください。")
            return
        selected_thresholds = self.selected_class_thresholds()
        if not selected_thresholds:
            QMessageBox.warning(self, "対象クラス", "再推論するクラスを1つ以上選択してください。")
            return
        low_thresholds = {
            name: threshold
            for name, threshold in selected_thresholds.items()
            if threshold < MIN_MAPPING_THRESHOLD
        }
        if low_thresholds:
            details = ", ".join(
                f"{name}={threshold:.2f}"
                for name, threshold in low_thresholds.items()
            )
            QMessageBox.warning(
                self,
                "閾値が低すぎます",
                f"地図生成では閾値{MIN_MAPPING_THRESHOLD:.2f}以上が必要です。\n"
                f"現在: {details}\n\n"
                "低い閾値はプレビューだけで使用してください。",
            )
            return
        class_text = ", ".join(
            f"{name}={threshold:.2f}"
            for name, threshold in selected_thresholds.items()
        )
        answer = QMessageBox.question(
            self,
            "部分再推論を開始",
            "SAM3 Dockerサーバーを再起動し、選択区間を処理します。\n"
            f"対象: {class_text}\n"
            "実行中のオンラインSAM3やUnity画像送信は停止してください。\n\n"
            "元地図は上書きしません。開始しますか？",
        )
        if answer != QMessageBox.StandardButton.Yes:
            return
        self.log.clear()
        self._stdout_buffer = ""
        self.output_directory = None
        self.open_button.setEnabled(False)
        self.run_button.setEnabled(False)
        self.stop_button.setEnabled(True)
        self.progress.setRange(0, 0)
        self.process = QProcess(self)
        self.process.setProgram("/usr/bin/setsid")
        arguments = [
            sys.executable,
            str(RUNNER),
            "--bag",
            self.bag_edit.text().strip(),
            "--base-map",
            self.map_edit.text().strip(),
            "--start",
            f"{self.start_spin.value():.6f}",
            "--end",
            f"{self.end_spin.value():.6f}",
            "--rate",
            self.rate_combo.currentText(),
        ]
        for class_name, threshold in selected_thresholds.items():
            arguments.extend(["--target-class", class_name])
            arguments.extend(
                ["--class-threshold", f"{class_name}={threshold:.3f}"]
            )
            arguments.extend(
                [
                    "--class-prompt",
                    f"{class_name}={self.class_prompt_values.get(class_name, class_name)}",
                ]
            )
        self.process.setArguments(arguments)
        self.process.setWorkingDirectory(str(APP_DIR))
        self.process.readyReadStandardOutput.connect(self.read_stdout)
        self.process.readyReadStandardError.connect(self.read_stderr)
        self.process.finished.connect(self.repair_finished)
        self.process.start()

    def read_stdout(self):
        text = bytes(self.process.readAllStandardOutput()).decode("utf-8", errors="replace")
        self._consume_output(text)

    def read_stderr(self):
        text = bytes(self.process.readAllStandardError()).decode("utf-8", errors="replace")
        self.log.appendPlainText(text.rstrip())

    def _consume_output(self, text: str):
        self.log.moveCursor(QTextCursor.MoveOperation.End)
        self.log.insertPlainText(text)
        self._stdout_buffer += text
        lines = self._stdout_buffer.splitlines(keepends=True)
        if lines and not lines[-1].endswith(("\n", "\r")):
            self._stdout_buffer = lines.pop()
        else:
            self._stdout_buffer = ""
        for line in lines:
            if line.startswith("SAM3_REPAIR_OUTPUT="):
                self.output_directory = Path(line.split("=", 1)[1].strip())

    def stop_repair(self):
        if self.process and self.process.state() != QProcess.ProcessState.NotRunning:
            try:
                os.killpg(int(self.process.processId()), signal.SIGINT)
            except (ProcessLookupError, PermissionError):
                self.process.terminate()
            self.log.appendPlainText("\n中止要求を送信しました。安全な終了を待っています…")

    def repair_finished(self, exit_code: int, _status):
        self.progress.setRange(0, 1)
        self.progress.setValue(1 if exit_code == 0 else 0)
        self.run_button.setEnabled(True)
        self.stop_button.setEnabled(False)
        if exit_code == 0 and self.output_directory and self.output_directory.is_dir():
            self.open_button.setEnabled(True)
            QMessageBox.information(
                self,
                "補正完了",
                f"新しい地図を保存しました。\n{self.output_directory}",
            )
        elif exit_code != 130:
            QMessageBox.critical(
                self,
                "補正失敗",
                "処理が完了しませんでした。ログを確認してください。元地図は変更していません。",
            )
        self.process = None

    def open_output(self):
        if self.output_directory:
            QDesktopServices.openUrl(QUrl.fromLocalFile(str(self.output_directory)))

    def closeEvent(self, event):
        if self.process and self.process.state() != QProcess.ProcessState.NotRunning:
            QMessageBox.warning(self, "処理中", "先に処理を中止してください。")
            event.ignore()
            return
        running_threads = [
            thread for thread in self.active_threads if thread.isRunning()
        ]
        if running_threads:
            self.close_after_threads = True
            self.pending_slider_preview = None
            self.sam_slider_timer.stop()
            self.playback_timer.stop()
            for thread in running_threads:
                thread.requestInterruption()
            self.score_label.setText("画像処理を安全に終了してから閉じます…")
            event.ignore()
            return
        self.clear_segment_preview_cache()
        event.accept()


def main() -> int:
    app = QApplication(sys.argv)
    app.setApplicationName("SAM3 Map Repair")
    window = SAM3MapRepairWindow()
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
