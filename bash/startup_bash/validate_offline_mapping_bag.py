#!/usr/bin/env python3
"""Validate that a rosbag contains the inputs required for offline SAM3 mapping."""

from __future__ import annotations

import argparse
from collections import defaultdict
from pathlib import Path
import sys

from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageFilter, StorageOptions
from rosidl_runtime_py.utilities import get_message


REQUIRED_TOPICS = {
    "/camera/stereo_sbs/compressed": "ステレオ画像",
    "/camera/stereo_params": "カメラ内部パラメータ",
    "/tf": "動的TF",
    "/tf_static": "静的TF",
    "/odom/filtered": "補正前オドメトリ",
    "/scan3": "LiDAR",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument(
        "--require-clock",
        action="store_true",
        help="Unityシミュレーション録画として /clock も必須にする",
    )
    return parser.parse_args()


def connected(graph: dict[str, set[str]], start: str, goal: str) -> bool:
    pending = [start]
    visited: set[str] = set()
    while pending:
        frame = pending.pop()
        if frame == goal:
            return True
        if frame in visited:
            continue
        visited.add(frame)
        pending.extend(graph.get(frame, set()) - visited)
    return False


def main() -> int:
    args = parse_args()
    if not args.bag.is_dir():
        print(f"✗ Rosbagディレクトリがありません: {args.bag}")
        return 2

    try:
        reader = SequentialReader()
        reader.open(
            StorageOptions(uri=str(args.bag), storage_id="mcap"),
            ConverterOptions("", ""),
        )
        metadata = reader.get_metadata()
    except Exception as exc:
        print(f"✗ Rosbagを開けません: {exc}")
        return 2

    topic_info = {
        item.topic_metadata.name: item for item in metadata.topics_with_message_count
    }
    duration_sec = metadata.duration.nanoseconds / 1e9
    failures: list[str] = []

    print("必須トピック:")
    required = dict(REQUIRED_TOPICS)
    if args.require_clock:
        required["/clock"] = "シミュレーション時刻"

    for topic, label in required.items():
        count = topic_info.get(topic).message_count if topic in topic_info else 0
        if count > 0:
            rate = count / duration_sec if duration_sec > 0 else 0.0
            print(f"  ✓ {topic}: {count}件 ({rate:.2f} Hz) - {label}")
        else:
            print(f"  ✗ {topic}: 0件 - {label}")
            failures.append(f"{topic} が録画されていません")

    camera_count = (
        topic_info["/camera/stereo_sbs/compressed"].message_count
        if "/camera/stereo_sbs/compressed" in topic_info
        else 0
    )
    camera_rate = camera_count / duration_sec if duration_sec > 0 else 0.0
    if camera_count and duration_sec >= 4.0 and camera_rate < 0.5:
        failures.append(f"カメラレートが低すぎます ({camera_rate:.2f} Hz)")
        print(f"  ✗ カメラ受信レート: {camera_rate:.2f} Hz (最低0.50 Hz)")

    tf_types = {
        topic: item.topic_metadata.type
        for topic, item in topic_info.items()
        if topic in ("/tf", "/tf_static")
    }
    graph: dict[str, set[str]] = defaultdict(set)
    corrected_tf_count = 0
    try:
        reader.set_filter(StorageFilter(topics=["/tf", "/tf_static"]))
        message_classes = {
            topic: get_message(type_name) for topic, type_name in tf_types.items()
        }
        while reader.has_next():
            topic, data, _ = reader.read_next()
            message = deserialize_message(data, message_classes[topic])
            for transform in message.transforms:
                parent = transform.header.frame_id.lstrip("/")
                child = transform.child_frame_id.lstrip("/")
                graph[parent].add(child)
                graph[child].add(parent)
                if parent == "map":
                    corrected_tf_count += 1
    except Exception as exc:
        failures.append(f"TF内容を読み取れません: {exc}")

    print("TF構成:")
    if corrected_tf_count > 0:
        print(f"  ✓ SLAM Toolbox補正TF: map親フレームを{corrected_tf_count}件確認")
    else:
        print("  ✗ SLAM Toolbox補正TF: map親フレームがありません")
        failures.append("SLAM Toolboxの補正済みTFがありません")

    required_chains = (
        ("map", "sirius3/base_footprint", "map → base_footprint"),
        (
            "sirius3/base_footprint",
            "sirius3/zed_camera_link",
            "base_footprint → zed_camera_link",
        ),
    )
    for start, goal, label in required_chains:
        if connected(graph, start, goal):
            print(f"  ✓ {label}")
        else:
            print(f"  ✗ {label} が接続されていません")
            failures.append(f"TF {label} が接続されていません")

    print("=" * 57)
    if failures:
        print("✗ このRosbagはSAM3オフラインマッピングに使用できません。")
        for failure in failures:
            print(f"  - {failure}")
        return 1

    print("✓ このRosbagにはオフラインマッピングの必須情報が入っています。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
