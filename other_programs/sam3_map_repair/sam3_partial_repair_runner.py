#!/usr/bin/env python3
"""Run a bounded rosbag mapping pass and merge selected semantic classes."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import traceback
import urllib.error
import urllib.request

from sam3_map_repair_core import (
    DEFAULT_CLASS_PROMPTS,
    MIN_MAPPING_THRESHOLD,
    SEMANTIC_CLASS_REGISTRY,
    inspect_bag,
    merge_semantic_classes_patch,
)


HOME = Path.home()
WORKSPACE = HOME / "sirius_jazzy_ws"
SERVER_DIRECTORY = HOME / "sam3_zed_server"
SERVER_URL = "http://localhost:8080"
MAP_SAVE_SCRIPT = WORKSPACE / "bash/startup_bash/rtabmap_save.sh"


def log(message: str) -> None:
    print(message, flush=True)


def http_json(endpoint: str, payload: dict | None = None, timeout: float = 5.0) -> dict:
    data = None if payload is None else json.dumps(payload).encode("utf-8")
    request = urllib.request.Request(
        SERVER_URL + endpoint,
        data=data,
        headers={"Content-Type": "application/json"},
        method="GET" if payload is None else "POST",
    )
    with urllib.request.urlopen(request, timeout=timeout) as response:
        return json.loads(response.read().decode("utf-8"))


def run_checked(
    command: list[str], env: dict | None = None, cwd: Path | None = None
) -> None:
    log("$ " + " ".join(command))
    completed = subprocess.run(command, env=env, cwd=cwd, check=False)
    if completed.returncode != 0:
        raise RuntimeError(
            f"コマンドが失敗しました (exit={completed.returncode}): {' '.join(command)}"
        )


def ensure_clean_server() -> None:
    log("[1/7] SAM3 GPUサーバーをクリーンな状態で起動します。")
    inspected = subprocess.run(
        ["docker", "container", "inspect", "sam3_zed_container"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    if inspected.returncode == 0:
        run_checked(["docker", "restart", "sam3_zed_container"])
    else:
        run_checked(
            ["docker", "compose", "up", "-d", "sam3-zed-merged"],
            env=os.environ.copy(),
            cwd=SERVER_DIRECTORY,
        )
    for _ in range(120):
        try:
            state = http_json("/debug_state", timeout=1.0)
            if "sam3_settings" in state:
                return
        except (OSError, ValueError, urllib.error.URLError):
            pass
        time.sleep(1)
    raise RuntimeError("SAM3サーバーが120秒以内に準備完了しませんでした。")


def configure_server(
    class_thresholds: dict[str, float], class_prompts: dict[str, str]
) -> None:
    prompt_thresholds = {
        class_prompts[name]: threshold
        for name, threshold in class_thresholds.items()
    }
    if len(prompt_thresholds) != len(class_thresholds):
        raise ValueError("複数クラスに同じSAM3推論文は使用できません。")
    prompt_registry = {
        class_prompts[name]: SEMANTIC_CLASS_REGISTRY[name]
        for name in class_thresholds
    }
    prompt = ", ".join(prompt_thresholds)
    setting_text = ", ".join(
        f"{name} <- '{class_prompts[name]}' ({value:.2f})"
        for name, value in class_thresholds.items()
    )
    log(f"[2/7] 意味クラス・SAM3推論文・閾値を設定します: {setting_text}")
    settings = (
        ("/class_registry", {"classes": prompt_registry}),
        ("/prompt", {"prompt": prompt}),
        ("/threshold", {"threshold": 0.5}),
        ("/class_thresholds", {"classes": prompt_thresholds}),
        ("/sam3_resolution", {"resolution": 512}),
        ("/color_mode", {"mode": "real"}),
        ("/fast_iters", {"iters": 4}),
        ("/depth_downsample", {"downsample": 4}),
        ("/max_distance", {"distance": 15.0}),
        ("/depth_mode", {"mode": "fast_stereo"}),
        ("/source_mode", {"mode": "network"}),
    )
    for endpoint, payload in settings:
        result = http_json(endpoint, payload)
        if not result.get("ok", False):
            raise RuntimeError(f"SAM3設定に失敗しました: {endpoint}: {result}")


def ros_command(arguments: list[str]) -> list[str]:
    quoted = subprocess.list2cmdline(arguments)
    return [
        "/bin/bash",
        "-lc",
        f"source {WORKSPACE}/install/setup.bash && exec {quoted}",
    ]


def start_mapping(env: dict) -> subprocess.Popen:
    log("[3/7] 部分マッピングノードを起動します。")
    command = ros_command(
        [
            "ros2",
            "launch",
            "sirius_navigation",
            "sam3_offline_mapping.launch.py",
            "use_sim_time:=true",
            "run_slam_toolbox:=false",
            "include_background:=true",
            "use_docker_backend:=true",
            "rviz:=false",
        ]
    )
    return subprocess.Popen(command, env=env, start_new_session=True)


def stop_process_group(process: subprocess.Popen | None) -> None:
    if process is None or process.poll() is not None:
        return
    try:
        # Signal the launch parent once; ros2 launch forwards it to children.
        # killpg() here would make every child receive SIGINT twice.
        process.send_signal(signal.SIGINT)
        process.wait(timeout=20)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass


def play_static_tf(bag: Path, env: dict) -> None:
    log("[4/7] bag先頭の /tf_static を先に読み込みます。")
    command = ros_command(
        [
            "ros2",
            "bag",
            "play",
            str(bag),
            "--clock",
            "--topics",
            "/tf_static",
            "--playback-duration",
            "1.0",
            "--disable-keyboard-controls",
        ]
    )
    run_checked(command, env=env)


def play_segment(
    bag: Path,
    start_seconds: float,
    end_seconds: float,
    rate: float,
    env: dict,
) -> None:
    warmup_start = max(0.0, start_seconds - 2.0)
    duration = end_seconds - warmup_start
    log(
        f"[5/7] {warmup_start:.2f}s～{end_seconds:.2f}sを{rate:.2f}倍速で再生します。"
    )
    log("       先頭2秒はTFと推論のウォームアップ範囲です。")
    command = ros_command(
        [
            "ros2",
            "bag",
            "play",
            str(bag),
            "--clock",
            "--start-offset",
            f"{warmup_start:.6f}",
            "--playback-duration",
            f"{duration:.6f}",
            "--rate",
            f"{rate:.6f}",
            "--disable-keyboard-controls",
        ]
    )
    run_checked(command, env=env)


def save_patch(bag: Path, start_seconds: float, end_seconds: float, env: dict) -> Path:
    log("[6/7] 部分推論結果をパッチ地図として保存します。")
    safe_range = f"{int(start_seconds):05d}_{int(end_seconds):05d}"
    requested_name = f"semantic_repair_{bag.name}_{safe_range}"
    with tempfile.NamedTemporaryFile(prefix="sam3-map-result-", delete=False) as stream:
        result_path = Path(stream.name)
    try:
        save_env = dict(env)
        save_env["SAM3_MAP_RESULT_FILE"] = str(result_path)
        run_checked(
            ["/bin/bash", str(MAP_SAVE_SCRIPT), requested_name],
            env=save_env,
        )
        if not result_path.is_file() or not result_path.read_text().strip():
            raise RuntimeError("パッチ地図の保存先を取得できませんでした。")
        patch_directory = Path(result_path.read_text().strip()).resolve()
        if not patch_directory.is_dir():
            raise RuntimeError(f"パッチ地図ディレクトリがありません: {patch_directory}")
        return patch_directory
    finally:
        result_path.unlink(missing_ok=True)


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True, type=Path)
    parser.add_argument("--base-map", required=True, type=Path)
    parser.add_argument("--start", required=True, type=float)
    parser.add_argument("--end", required=True, type=float)
    parser.add_argument("--target-class", action="append", dest="target_classes")
    parser.add_argument(
        "--class-threshold",
        action="append",
        default=[],
        metavar="CLASS=VALUE",
    )
    parser.add_argument(
        "--class-prompt",
        action="append",
        default=[],
        metavar="CLASS=PROMPT",
    )
    parser.add_argument("--threshold", default=0.4, type=float)
    parser.add_argument("--rate", default=0.25, type=float)
    parser.add_argument("--domain-id", default=43, type=int)
    return parser.parse_args()


def main() -> int:
    args = parse_arguments()
    mapping_process = None
    try:
        target_classes = args.target_classes or ["tactile paving"]
        class_thresholds = {name: float(args.threshold) for name in target_classes}
        class_prompts = {
            name: DEFAULT_CLASS_PROMPTS.get(name, name) for name in target_classes
        }
        for assignment in args.class_threshold:
            if "=" not in assignment:
                raise ValueError(f"クラス閾値の形式が不正です: {assignment}")
            name, raw_value = assignment.rsplit("=", 1)
            if name not in class_thresholds:
                raise ValueError(f"未選択クラスの閾値です: {name}")
            class_thresholds[name] = float(raw_value)
        for assignment in args.class_prompt:
            if "=" not in assignment:
                raise ValueError(f"クラス推論文の形式が不正です: {assignment}")
            name, prompt = assignment.split("=", 1)
            prompt = " ".join(prompt.strip().split())
            if name not in class_prompts:
                raise ValueError(f"未選択クラスの推論文です: {name}")
            if not prompt or "," in prompt:
                raise ValueError(f"クラス推論文が不正です: {assignment}")
            class_prompts[name] = prompt
        summary = inspect_bag(args.bag)
        if summary.missing_topics:
            raise ValueError(
                "rosbag必須トピック不足: " + ", ".join(sorted(summary.missing_topics))
            )
        if not 0 <= args.start < args.end <= summary.duration_seconds + 0.01:
            raise ValueError(
                f"時間範囲が不正です。bag長={summary.duration_seconds:.2f}s"
            )
        if (
            any(not 0.0 <= value <= 1.0 for value in class_thresholds.values())
            or args.rate <= 0
        ):
            raise ValueError("閾値または再生速度が不正です。")
        if any(value < MIN_MAPPING_THRESHOLD for value in class_thresholds.values()):
            raise ValueError(
                f"地図補正のSAM3閾値は{MIN_MAPPING_THRESHOLD:.2f}以上にしてください。"
            )

        env = os.environ.copy()
        env["ROS_DOMAIN_ID"] = str(args.domain_id)
        ensure_clean_server()
        configure_server(class_thresholds, class_prompts)
        mapping_process = start_mapping(env)
        time.sleep(6)
        if mapping_process.poll() is not None:
            raise RuntimeError("部分マッピングlaunchが起動直後に終了しました。")
        play_static_tf(summary.path, env)
        play_segment(summary.path, args.start, args.end, args.rate, env)
        log("GPU/RTAB-Mapの残処理を待機しています...")
        time.sleep(8)
        patch_directory = save_patch(
            summary.path, args.start, args.end, env
        )
        log("[7/7] 元地図を残し、選択クラスだけを新しい地図へ追加します。")
        result = merge_semantic_classes_patch(
            args.base_map,
            patch_directory,
            class_thresholds,
            summary.path,
            args.start,
            args.end,
            class_prompts=class_prompts,
        )
        for class_name, count in (result.added_cells_by_class or {}).items():
            log(f"  {class_name}: {count}セル追加")
        log(f"追加セル合計: {result.added_cells}")
        log(f"SAM3_REPAIR_OUTPUT={result.output_directory}")
        log("✓ 部分補正が完了しました。元地図は変更していません。")
        return 0
    except KeyboardInterrupt:
        log("中止しました。元地図は変更していません。")
        return 130
    except Exception as error:
        log(f"エラー: {error}")
        traceback.print_exc()
        return 1
    finally:
        stop_process_group(mapping_process)


if __name__ == "__main__":
    raise SystemExit(main())
