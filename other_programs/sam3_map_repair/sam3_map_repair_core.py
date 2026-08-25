#!/usr/bin/env python3
"""Core helpers for selective SAM3 rosbag replay and semantic-map merging."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
import json
import math
from pathlib import Path
import shutil
from typing import Any

import cv2
import numpy as np
import yaml


CAMERA_TOPIC = "/camera/stereo_sbs/compressed"
PARAMS_TOPIC = "/camera/stereo_params"
REQUIRED_TOPICS = {CAMERA_TOPIC, PARAMS_TOPIC, "/tf", "/tf_static"}
DEFAULT_CAMERA_PARAMS = {
    "fx": 448.14,
    "fy": 448.14,
    "cx": 640.0,
    "cy": 360.0,
    "baseline": 0.12,
}
MIN_MAPPING_THRESHOLD = 0.20
MAX_PATCH_CLASS_COVERAGE = {"tactile paving": 0.35}
MIN_COVERAGE_CHECK_CELLS = 500
SEMANTIC_CLASS_REGISTRY = {
    "grass": {"id": 3, "color": [0, 255, 0]},
    "tactile paving": {"id": 4, "color": [255, 255, 0]},
    "roadway": {"id": 5, "color": [0, 0, 255]},
    "sidewalk": {"id": 6, "color": [128, 128, 128]},
}
DEFAULT_CLASS_PROMPTS = {
    "grass": "grass",
    "tactile paving": "line type tactile paving",
    "roadway": "roadway",
    "sidewalk": "sidewalk",
}


@dataclass(frozen=True)
class BagSummary:
    path: Path
    duration_seconds: float
    start_nanoseconds: int
    storage_id: str
    topics: frozenset[str]

    @property
    def missing_topics(self) -> set[str]:
        return REQUIRED_TOPICS - set(self.topics)


@dataclass(frozen=True)
class CameraFrame:
    jpeg: bytes
    bag_offset_seconds: float
    stamp_seconds: float
    camera_params: dict[str, float]


@dataclass(frozen=True)
class MergeResult:
    output_directory: Path
    added_cells: int
    target_class_id: int
    target_class_name: str
    added_cells_by_class: dict[str, int] | None = None
    target_class_ids: dict[str, int] | None = None


def _load_yaml(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise ValueError(f"YAMLが不正です: {path}")
    return data


def inspect_bag(path: str | Path) -> BagSummary:
    bag_path = Path(path).expanduser().resolve()
    metadata_path = bag_path / "metadata.yaml"
    if not bag_path.is_dir() or not metadata_path.is_file():
        raise FileNotFoundError(f"metadata.yamlを含むrosbagではありません: {bag_path}")
    metadata = _load_yaml(metadata_path).get("rosbag2_bagfile_information", {})
    topics = frozenset(
        item.get("topic_metadata", {}).get("name", "")
        for item in metadata.get("topics_with_message_count", [])
    )
    duration_ns = int(metadata.get("duration", {}).get("nanoseconds", 0))
    start_ns = int(metadata.get("starting_time", {}).get("nanoseconds_since_epoch", 0))
    return BagSummary(
        path=bag_path,
        duration_seconds=duration_ns / 1e9,
        start_nanoseconds=start_ns,
        storage_id=str(metadata.get("storage_identifier", "mcap")),
        topics=topics,
    )


def read_camera_frame(path: str | Path, offset_seconds: float) -> CameraFrame:
    """Seek close to an offset and deserialize the first usable stereo frame."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    summary = inspect_bag(path)
    if summary.missing_topics & {CAMERA_TOPIC, PARAMS_TOPIC}:
        raise ValueError(
            "カメラトピックが不足しています: "
            + ", ".join(sorted(summary.missing_topics))
        )
    offset_seconds = min(max(0.0, float(offset_seconds)), summary.duration_seconds)
    seek_offset = max(0.0, offset_seconds - 3.0)

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(
            uri=str(summary.path), storage_id=summary.storage_id
        ),
        rosbag2_py.ConverterOptions("", ""),
    )
    type_map = {
        item.name: item.type for item in reader.get_all_topics_and_types()
    }
    reader.set_filter(
        rosbag2_py.StorageFilter(topics=[CAMERA_TOPIC, PARAMS_TOPIC])
    )
    reader.seek(summary.start_nanoseconds + int(seek_offset * 1e9))

    params = dict(DEFAULT_CAMERA_PARAMS)
    params_received = False
    target_ns = summary.start_nanoseconds + int(offset_seconds * 1e9)
    fallback_frame = None
    while reader.has_next():
        topic, serialized, timestamp_ns = reader.read_next()
        if topic == PARAMS_TOPIC:
            msg = deserialize_message(serialized, get_message(type_map[topic]))
            try:
                decoded = json.loads(msg.data)
                for key in DEFAULT_CAMERA_PARAMS:
                    if key in decoded:
                        params[key] = float(decoded[key])
                params_received = True
            except (TypeError, ValueError, json.JSONDecodeError):
                pass
            continue
        if topic != CAMERA_TOPIC:
            continue
        msg = deserialize_message(serialized, get_message(type_map[topic]))
        fallback_frame = (bytes(msg.data), msg, timestamp_ns)
        if timestamp_ns >= target_ns and params_received:
            break
        if timestamp_ns > target_ns + int(5e9):
            break

    if fallback_frame is None:
        raise RuntimeError("指定位置付近にステレオ画像がありません。")
    jpeg, msg, timestamp_ns = fallback_frame
    stamp_seconds = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    return CameraFrame(
        jpeg=jpeg,
        bag_offset_seconds=(timestamp_ns - summary.start_nanoseconds) / 1e9,
        stamp_seconds=stamp_seconds,
        camera_params=params,
    )


def find_map_base(directory: str | Path) -> Path:
    map_directory = Path(directory).expanduser().resolve()
    if not map_directory.is_dir():
        raise FileNotFoundError(f"地図ディレクトリがありません: {map_directory}")
    candidates = sorted(map_directory.glob("*.colored.json"))
    for json_path in candidates:
        base = json_path.with_name(json_path.name[: -len(".colored.json")])
        if (
            base.with_suffix(".yaml").is_file()
            and base.with_suffix(".pgm").is_file()
            and base.with_name(base.name + ".colored.pgm").is_file()
        ):
            return base
    raise FileNotFoundError(
        f".yaml/.pgm/.colored.pgm/.colored.jsonが揃った地図がありません: {map_directory}"
    )


def _pixel_to_world(pixel: np.ndarray, meta: dict, height: int) -> np.ndarray:
    resolution = float(meta["resolution"])
    origin = meta["origin"]
    yaw = float(origin[2]) if len(origin) > 2 else 0.0
    local = np.array(
        [(pixel[0] + 0.5) * resolution, (height - pixel[1] - 0.5) * resolution]
    )
    rotation = np.array(
        [[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]]
    )
    return rotation @ local + np.asarray(origin[:2], dtype=np.float64)


def _world_to_pixel(world: np.ndarray, meta: dict, height: int) -> np.ndarray:
    resolution = float(meta["resolution"])
    origin = meta["origin"]
    yaw = float(origin[2]) if len(origin) > 2 else 0.0
    inverse = np.array(
        [[math.cos(yaw), math.sin(yaw)], [-math.sin(yaw), math.cos(yaw)]]
    )
    local = inverse @ (world - np.asarray(origin[:2], dtype=np.float64))
    return np.array(
        [local[0] / resolution - 0.5, height - local[1] / resolution - 0.5]
    )


def map_affine(
    source_meta: dict,
    source_height: int,
    destination_meta: dict,
    destination_height: int,
) -> np.ndarray:
    source = np.float32([[0, 0], [1, 0], [0, 1]])
    destination = np.float32(
        [
            _world_to_pixel(
                _pixel_to_world(point, source_meta, source_height),
                destination_meta,
                destination_height,
            )
            for point in source
        ]
    )
    return cv2.getAffineTransform(source, destination)


def _class_id(metadata: dict, class_name: str) -> int:
    for raw_id, info in metadata.get("labels", {}).items():
        if str(info.get("name", "")).strip().lower() == class_name.strip().lower():
            return int(raw_id)
    raise KeyError(f"地図にクラス定義がありません: {class_name}")


def _unique_directory(candidate: Path) -> Path:
    if not candidate.exists():
        return candidate
    index = 2
    while True:
        alternative = candidate.with_name(f"{candidate.name}_{index:02d}")
        if not alternative.exists():
            return alternative
        index += 1


def _copy_versioned_map(source_base: Path, output_directory: Path) -> Path:
    output_directory.mkdir(parents=True)
    new_name = output_directory.name
    for source in source_base.parent.iterdir():
        if not source.is_file():
            continue
        destination_name = source.name
        if destination_name.startswith(source_base.name):
            destination_name = new_name + destination_name[len(source_base.name) :]
        shutil.copy2(source, output_directory / destination_name)
    output_base = output_directory / new_name
    yaml_path = output_base.with_suffix(".yaml")
    if yaml_path.is_file():
        yaml_data = _load_yaml(yaml_path)
        yaml_data["image"] = output_base.with_suffix(".pgm").name
        with yaml_path.open("w", encoding="utf-8") as stream:
            yaml.safe_dump(yaml_data, stream, sort_keys=False, allow_unicode=True)
    return output_base


def merge_semantic_classes_patch(
    base_directory: str | Path,
    patch_directory: str | Path,
    class_thresholds: dict[str, float],
    bag_path: str | Path,
    start_seconds: float,
    end_seconds: float,
    output_directory: str | Path | None = None,
    class_prompts: dict[str, str] | None = None,
) -> MergeResult:
    """Add selected target classes from a partial map to a versioned base map."""
    if not class_thresholds:
        raise ValueError("補正対象クラスが選択されていません。")
    target_classes = list(class_thresholds)
    base_source = find_map_base(base_directory)
    patch_base = find_map_base(patch_directory)
    low_thresholds = {
        name: float(value)
        for name, value in class_thresholds.items()
        if float(value) < MIN_MAPPING_THRESHOLD
    }
    if low_thresholds:
        details = ", ".join(
            f"{name}={value:.2f}" for name, value in low_thresholds.items()
        )
        raise ValueError(
            f"地図補正のSAM3閾値は{MIN_MAPPING_THRESHOLD:.2f}以上が必要です: "
            f"{details}。低い閾値はプレビューだけで使用してください。"
        )

    patch_index_check = cv2.imread(
        str(patch_base.with_name(patch_base.name + ".colored.pgm")),
        cv2.IMREAD_GRAYSCALE,
    )
    with patch_base.with_name(patch_base.name + ".colored.json").open(
        "r", encoding="utf-8"
    ) as stream:
        patch_meta_check = json.load(stream)
    if patch_index_check is None:
        raise ValueError("部分意味地図を読み込めません。")
    observed_cells = int(np.count_nonzero(patch_index_check != 0))
    if observed_cells >= MIN_COVERAGE_CHECK_CELLS:
        for class_name, maximum in MAX_PATCH_CLASS_COVERAGE.items():
            if class_name not in class_thresholds:
                continue
            class_id = _class_id(patch_meta_check, class_name)
            target_cells = int(np.count_nonzero(patch_index_check == class_id))
            coverage = target_cells / observed_cells
            if coverage > maximum:
                raise ValueError(
                    f"安全停止: {class_name} が部分地図の観測領域の"
                    f"{coverage * 100:.1f}%を占めています"
                    f"（上限{maximum * 100:.0f}%）。"
                    "閾値を上げ、区間SAM3プレビューで誤検出範囲を確認してください。"
                )
    if output_directory is None:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        class_slug = (
            target_classes[0].replace(" ", "_")
            if len(target_classes) == 1
            else "multi"
        )
        output_directory = base_source.parent.parent / (
            f"{base_source.parent.name}_repair_{class_slug}_{stamp}"
        )
    output_directory = _unique_directory(Path(output_directory).expanduser().resolve())
    output_base = _copy_versioned_map(base_source, output_directory)

    base_yaml = _load_yaml(output_base.with_suffix(".yaml"))
    patch_yaml = _load_yaml(patch_base.with_suffix(".yaml"))
    base_index_path = output_base.with_name(output_base.name + ".colored.pgm")
    patch_index_path = patch_base.with_name(patch_base.name + ".colored.pgm")
    base_json_path = output_base.with_name(output_base.name + ".colored.json")
    patch_json_path = patch_base.with_name(patch_base.name + ".colored.json")
    base_index = cv2.imread(str(base_index_path), cv2.IMREAD_GRAYSCALE)
    patch_index = cv2.imread(str(patch_index_path), cv2.IMREAD_GRAYSCALE)
    structural = cv2.imread(str(output_base.with_suffix(".pgm")), cv2.IMREAD_GRAYSCALE)
    if base_index is None or patch_index is None or structural is None:
        raise ValueError("意味地図または構造PGMを読み込めません。")
    with base_json_path.open("r", encoding="utf-8") as stream:
        base_meta = json.load(stream)
    with patch_json_path.open("r", encoding="utf-8") as stream:
        patch_semantic_meta = json.load(stream)
    base_class_ids = {
        class_name: _class_id(base_meta, class_name)
        for class_name in target_classes
    }
    patch_class_ids = {
        class_name: _class_id(patch_semantic_meta, class_name)
        for class_name in target_classes
    }

    affine = map_affine(
        patch_yaml,
        patch_index.shape[0],
        base_yaml,
        base_index.shape[0],
    )
    warped = cv2.warpAffine(
        patch_index,
        affine,
        (base_index.shape[1], base_index.shape[0]),
        flags=cv2.INTER_NEAREST,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=0,
    )
    negate = int(base_yaml.get("negate", 0))
    pixel_probability = structural.astype(np.float32) / 255.0
    occupied_probability = pixel_probability if negate else 1.0 - pixel_probability
    free = occupied_probability <= float(base_yaml.get("free_thresh", 0.196))
    added_masks = {}
    added_cells_by_class = {}
    for class_name in target_classes:
        base_class_id = base_class_ids[class_name]
        patch_class_id = patch_class_ids[class_name]
        add_mask = (warped == patch_class_id) & free & (base_index != 1)
        previous_target = base_index == base_class_id
        added_masks[class_name] = add_mask
        added_cells_by_class[class_name] = int(
            np.count_nonzero(add_mask & ~previous_target)
        )
        base_index[add_mask] = base_class_id
    added_cells = sum(added_cells_by_class.values())
    cv2.imwrite(str(base_index_path), base_index)

    palette = np.asarray(base_meta.get("palette", []), dtype=np.uint8)
    if palette.ndim == 2 and palette.shape[1] == 3 and len(palette):
        semantic_rgb = np.zeros((*base_index.shape, 3), dtype=np.uint8)
        valid = base_index < len(palette)
        semantic_rgb[valid] = palette[base_index[valid]]
        cv2.imwrite(
            str(output_base.with_name(output_base.name + ".semantic_full.png")),
            semantic_rgb[:, :, ::-1],
        )

    color_path = output_base.with_name(output_base.name + ".color.png")
    color = cv2.imread(str(color_path), cv2.IMREAD_COLOR)
    if color is not None and color.shape[:2] == base_index.shape:
        overlay = color.copy()
        for class_name, add_mask in added_masks.items():
            if not np.any(add_mask):
                continue
            class_id = base_class_ids[class_name]
            class_rgb = (
                palette[class_id]
                if len(palette) > class_id
                else np.array([255, 255, 0], dtype=np.uint8)
            )
            class_bgr = class_rgb[::-1].astype(np.float32)
            overlay[add_mask] = np.clip(
                color[add_mask].astype(np.float32) * 0.35 + class_bgr * 0.65,
                0,
                255,
            ).astype(np.uint8)
        cv2.imwrite(
            str(output_base.with_name(output_base.name + ".repair_overlay.png")),
            overlay,
        )

    history = list(base_meta.get("repair_history", []))
    history.append(
        {
            "created_at": datetime.now().isoformat(timespec="seconds"),
            "target_classes": target_classes,
            "class_thresholds": {
                name: float(value) for name, value in class_thresholds.items()
            },
            "class_prompts": dict(class_prompts or {}),
            "bag": str(Path(bag_path).expanduser().resolve()),
            "start_seconds": float(start_seconds),
            "end_seconds": float(end_seconds),
            "patch_map": str(patch_base.resolve()),
            "merge_mode": "add_only",
            "added_cells": added_cells,
            "added_cells_by_class": added_cells_by_class,
        }
    )
    base_meta["repair_history"] = history
    base_meta["width"] = int(base_index.shape[1])
    base_meta["height"] = int(base_index.shape[0])
    with base_json_path.open("w", encoding="utf-8") as stream:
        json.dump(base_meta, stream, ensure_ascii=False, indent=2)

    return MergeResult(
        output_directory=output_directory,
        added_cells=added_cells,
        target_class_id=base_class_ids[target_classes[0]],
        target_class_name=", ".join(target_classes),
        added_cells_by_class=added_cells_by_class,
        target_class_ids=base_class_ids,
    )


def merge_semantic_patch(
    base_directory: str | Path,
    patch_directory: str | Path,
    target_class: str,
    bag_path: str | Path,
    start_seconds: float,
    end_seconds: float,
    threshold: float,
    output_directory: str | Path | None = None,
) -> MergeResult:
    """Backward-compatible one-class wrapper."""
    return merge_semantic_classes_patch(
        base_directory,
        patch_directory,
        {target_class: threshold},
        bag_path,
        start_seconds,
        end_seconds,
        output_directory,
    )
