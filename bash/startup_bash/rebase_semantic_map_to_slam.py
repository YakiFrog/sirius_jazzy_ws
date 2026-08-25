#!/usr/bin/env python3
"""Place an offline SAM3 map on a SLAM Toolbox occupancy grid.

The semantic/texture products are transformed through the ROS map world frame.
The SLAM Toolbox PGM remains authoritative for occupied, free and unknown cells.
"""

from __future__ import annotations

import argparse
import json
import math
import shutil
from pathlib import Path

import cv2
import numpy as np
import yaml


def load_yaml(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict):
        raise ValueError(f"YAMLの内容が不正です: {path}")
    return data


def image_path_from_yaml(yaml_path: Path, metadata: dict) -> Path:
    image = Path(str(metadata.get("image", ""))).expanduser()
    if not image.is_absolute():
        image = yaml_path.parent / image
    return image.resolve()


def occupancy_masks(image: np.ndarray, metadata: dict) -> tuple[np.ndarray, np.ndarray]:
    """Return occupied and free masks using nav_msgs map YAML thresholds."""
    negate = int(metadata.get("negate", 0))
    pixels = image.astype(np.float32) / 255.0
    occupied_probability = pixels if negate else 1.0 - pixels
    occupied = occupied_probability >= float(metadata.get("occupied_thresh", 0.65))
    free = occupied_probability <= float(metadata.get("free_thresh", 0.196))
    return occupied, free


def pixel_to_world(pixel: np.ndarray, metadata: dict, height: int) -> np.ndarray:
    resolution = float(metadata["resolution"])
    origin = metadata["origin"]
    yaw = float(origin[2]) if len(origin) > 2 else 0.0
    local = np.array(
        [
            (float(pixel[0]) + 0.5) * resolution,
            (height - float(pixel[1]) - 0.5) * resolution,
        ],
        dtype=np.float64,
    )
    rotation = np.array(
        [[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]],
        dtype=np.float64,
    )
    return rotation @ local + np.asarray(origin[:2], dtype=np.float64)


def world_to_pixel(world: np.ndarray, metadata: dict, height: int) -> np.ndarray:
    resolution = float(metadata["resolution"])
    origin = metadata["origin"]
    yaw = float(origin[2]) if len(origin) > 2 else 0.0
    rotation_inverse = np.array(
        [[math.cos(yaw), math.sin(yaw)], [-math.sin(yaw), math.cos(yaw)]],
        dtype=np.float64,
    )
    local = rotation_inverse @ (world - np.asarray(origin[:2], dtype=np.float64))
    return np.array(
        [local[0] / resolution - 0.5, height - local[1] / resolution - 0.5],
        dtype=np.float64,
    )


def source_to_destination_affine(
    source_metadata: dict,
    source_height: int,
    destination_metadata: dict,
    destination_height: int,
) -> np.ndarray:
    source_points = np.float32([[0, 0], [1, 0], [0, 1]])
    destination_points = np.float32(
        [
            world_to_pixel(
                pixel_to_world(point, source_metadata, source_height),
                destination_metadata,
                destination_height,
            )
            for point in source_points
        ]
    )
    return cv2.getAffineTransform(source_points, destination_points)


def warp_nearest(
    image: np.ndarray,
    affine: np.ndarray,
    width: int,
    height: int,
    border_value: int | tuple[int, ...] = 0,
) -> np.ndarray:
    return cv2.warpAffine(
        image,
        affine,
        (width, height),
        flags=cv2.INTER_NEAREST,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=border_value,
    )


def unique_output_directory(candidate: Path) -> Path:
    if not candidate.exists():
        return candidate
    index = 2
    while True:
        alternative = candidate.with_name(f"{candidate.name}_{index:02d}")
        if not alternative.exists():
            return alternative
        index += 1


def rebase(source_base: Path, slam_yaml: Path, output_directory: Path | None) -> Path:
    source_yaml = source_base.with_suffix(".yaml")
    source_pgm = source_base.with_suffix(".pgm")
    source_indexed = source_base.parent / f"{source_base.name}.colored.pgm"
    source_json = source_base.parent / f"{source_base.name}.colored.json"
    source_color = source_base.parent / f"{source_base.name}.color.png"
    source_texture = source_base.parent / f"{source_base.name}.texture.png"

    required = [source_yaml, source_pgm, source_indexed, source_json, source_color, slam_yaml]
    missing = [str(path) for path in required if not path.is_file()]
    if missing:
        raise FileNotFoundError("必要なファイルがありません:\n  " + "\n  ".join(missing))

    source_metadata = load_yaml(source_yaml)
    destination_metadata = load_yaml(slam_yaml)
    slam_pgm = image_path_from_yaml(slam_yaml, destination_metadata)
    if not slam_pgm.is_file():
        raise FileNotFoundError(f"SLAM Toolbox PGMがありません: {slam_pgm}")

    source_grid = cv2.imread(str(source_pgm), cv2.IMREAD_GRAYSCALE)
    destination_grid = cv2.imread(str(slam_pgm), cv2.IMREAD_GRAYSCALE)
    indexed_grid = cv2.imread(str(source_indexed), cv2.IMREAD_GRAYSCALE)
    color_grid = cv2.imread(str(source_color), cv2.IMREAD_COLOR)
    if any(item is None for item in (source_grid, destination_grid, indexed_grid, color_grid)):
        raise ValueError("PGM/PNG画像の読み込みに失敗しました。")
    if indexed_grid.shape != source_grid.shape or color_grid.shape[:2] != source_grid.shape:
        raise ValueError("RTAB構造地図・意味地図・カラー地図のサイズが一致しません。")

    with source_json.open("r", encoding="utf-8") as stream:
        semantic_metadata = json.load(stream)
    if semantic_metadata.get("semantic_encoding") != "class_id":
        raise ValueError("colored.pgmがsemantic class_id形式ではありません。")

    source_occupied, source_free = occupancy_masks(source_grid, source_metadata)
    destination_occupied, destination_free = occupancy_masks(
        destination_grid, destination_metadata
    )
    source_height, source_width = source_grid.shape
    destination_height, destination_width = destination_grid.shape
    affine = source_to_destination_affine(
        source_metadata,
        source_height,
        destination_metadata,
        destination_height,
    )

    warped_source_free = warp_nearest(
        source_free.astype(np.uint8), affine, destination_width, destination_height
    ).astype(bool)
    warped_indexed = warp_nearest(
        indexed_grid, affine, destination_width, destination_height
    )
    warped_color = warp_nearest(
        color_grid, affine, destination_width, destination_height, (0, 0, 0)
    )

    # The laser occupancy grid is always authoritative for structure.
    destination_indexed = np.zeros_like(destination_grid, dtype=np.uint8)
    destination_indexed[destination_occupied] = 1
    destination_indexed[destination_free] = 2
    semantic_overlay = destination_free & warped_source_free & (warped_indexed > 2)
    destination_indexed[semantic_overlay] = warped_indexed[semantic_overlay]

    destination_color = np.full(
        (destination_height, destination_width, 3), 205, dtype=np.uint8
    )
    destination_color[destination_occupied] = (0, 0, 0)
    destination_color[destination_free] = (255, 255, 255)
    color_overlay = destination_free & warped_source_free
    destination_color[color_overlay] = warped_color[color_overlay]

    destination_texture = np.zeros(
        (destination_height, destination_width, 4), dtype=np.uint8
    )
    if source_texture.is_file():
        texture = cv2.imread(str(source_texture), cv2.IMREAD_UNCHANGED)
        if texture is not None and texture.shape[:2] == source_grid.shape and texture.shape[2] == 4:
            warped_texture = warp_nearest(
                texture, affine, destination_width, destination_height, (0, 0, 0, 0)
            )
            texture_overlay = destination_free & (warped_texture[:, :, 3] > 0)
            destination_texture[texture_overlay] = warped_texture[texture_overlay]

    if output_directory is None:
        output_directory = source_base.parent.parent / (
            f"{source_base.parent.name}_slam_base_{slam_yaml.stem}"
        )
    output_directory = unique_output_directory(output_directory.expanduser().resolve())
    output_directory.mkdir(parents=True)
    output_base = output_directory / output_directory.name

    output_pgm = output_base.with_suffix(".pgm")
    output_yaml = output_base.with_suffix(".yaml")
    output_indexed = output_directory / f"{output_base.name}.colored.pgm"
    output_json = output_directory / f"{output_base.name}.colored.json"
    output_color = output_directory / f"{output_base.name}.color.png"
    output_texture = output_directory / f"{output_base.name}.texture.png"
    output_semantic = output_directory / f"{output_base.name}.semantic_full.png"

    shutil.copy2(slam_pgm, output_pgm)
    output_map_metadata = dict(destination_metadata)
    output_map_metadata["image"] = output_pgm.name
    with output_yaml.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(output_map_metadata, stream, sort_keys=False, allow_unicode=True)
    cv2.imwrite(str(output_indexed), destination_indexed)
    cv2.imwrite(str(output_color), destination_color)
    cv2.imwrite(str(output_texture), destination_texture)

    palette = np.asarray(semantic_metadata.get("palette", []), dtype=np.uint8)
    if palette.ndim == 2 and palette.shape[1] == 3 and len(palette):
        semantic_visual = np.zeros(
            (destination_height, destination_width, 3), dtype=np.uint8
        )
        valid_ids = destination_indexed < len(palette)
        semantic_visual[valid_ids] = palette[destination_indexed[valid_ids]]
        cv2.imwrite(str(output_semantic), semantic_visual)

    output_semantic_metadata = dict(semantic_metadata)
    output_semantic_metadata.update(
        {
            "resolution": float(destination_metadata["resolution"]),
            "origin": [float(value) for value in destination_metadata["origin"][:2]],
            "width": destination_width,
            "height": destination_height,
            "structural_base": "slam_toolbox",
            "structural_map_yaml": str(slam_yaml.resolve()),
            "semantic_source_map": str(source_base.resolve()),
        }
    )
    with output_json.open("w", encoding="utf-8") as stream:
        json.dump(output_semantic_metadata, stream, ensure_ascii=False, indent=2)

    source_ply = source_base.with_suffix(".ply")
    if source_ply.is_file():
        shutil.copy2(source_ply, output_base.with_suffix(".ply"))

    print(f"SLAM_BASE_OUTPUT={output_directory}")
    print(f"  SLAM壁セル: {int(np.count_nonzero(destination_occupied))}")
    print(f"  移植した路面カラーセル: {int(np.count_nonzero(color_overlay))}")
    print(f"  移植した意味セル: {int(np.count_nonzero(semantic_overlay))}")
    return output_directory


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="SAM3地図をSLAM ToolboxのPGM構造地図へ世界座標で移植します。"
    )
    parser.add_argument("source_base", type=Path, help="RTAB/SAM3地図の拡張子なしパス")
    parser.add_argument("slam_yaml", type=Path, help="SLAM Toolbox地図YAML")
    parser.add_argument("--output-dir", type=Path, default=None, help="出力ディレクトリ")
    return parser.parse_args()


def main() -> int:
    arguments = parse_arguments()
    try:
        rebase(arguments.source_base.expanduser(), arguments.slam_yaml.expanduser(), arguments.output_dir)
    except Exception as error:
        print(f"エラー: SLAM Toolboxベース地図を生成できません: {error}")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
