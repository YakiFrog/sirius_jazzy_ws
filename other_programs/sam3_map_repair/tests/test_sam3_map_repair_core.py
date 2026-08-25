import importlib.util
from pathlib import Path
import sys

import cv2
import json
import numpy as np
import yaml


MODULE_PATH = Path(__file__).parents[1] / "sam3_map_repair_core.py"
SPEC = importlib.util.spec_from_file_location("sam3_map_repair_core", MODULE_PATH)
CORE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = CORE
SPEC.loader.exec_module(CORE)


def write_map(directory, name, grid, indexed, origin=(0.0, 0.0), resolution=1.0):
    directory.mkdir(parents=True)
    base = directory / name
    cv2.imwrite(str(base.with_suffix(".pgm")), grid)
    cv2.imwrite(str(base.with_name(base.name + ".colored.pgm")), indexed)
    cv2.imwrite(str(base.with_name(base.name + ".color.png")), np.dstack([grid] * 3))
    with base.with_suffix(".yaml").open("w") as stream:
        yaml.safe_dump(
            {
                "image": base.with_suffix(".pgm").name,
                "resolution": resolution,
                "origin": [origin[0], origin[1], 0.0],
                "negate": 0,
                "occupied_thresh": 0.65,
                "free_thresh": 0.196,
            },
            stream,
        )
    meta = {
        "semantic_encoding": "class_id",
        "palette": [[127, 127, 127], [0, 0, 0], [255, 255, 255], [0, 255, 0], [255, 255, 0]],
        "labels": {
            "0": {"name": "unknown"},
            "1": {"name": "wall"},
            "2": {"name": "floor"},
            "3": {"name": "grass"},
            "4": {"name": "tactile paving"},
        },
    }
    with base.with_name(base.name + ".colored.json").open("w") as stream:
        json.dump(meta, stream)
    return base


def test_merge_adds_target_and_preserves_wall(tmp_path):
    base_grid = np.full((6, 6), 254, np.uint8)
    base_grid[3, 2] = 0
    base_index = np.full((6, 6), 2, np.uint8)
    base_index[3, 2] = 1
    base = write_map(tmp_path / "base", "base", base_grid, base_index)

    patch_grid = np.full((3, 3), 254, np.uint8)
    patch_index = np.full((3, 3), 2, np.uint8)
    patch_index[1, 1] = 4
    patch_index[2, 2] = 4
    patch = write_map(tmp_path / "patch", "patch", patch_grid, patch_index, origin=(1, 1))

    result = CORE.merge_semantic_patch(
        base.parent,
        patch.parent,
        "tactile paving",
        "/tmp/bag",
        10,
        20,
        0.4,
        tmp_path / "result",
    )
    merged_base = CORE.find_map_base(result.output_directory)
    merged = cv2.imread(str(merged_base.with_name(merged_base.name + ".colored.pgm")), 0)
    assert merged[3, 2] == 1
    assert merged[4, 3] == 4
    assert result.added_cells == 1


def test_affine_translation():
    source = {"resolution": 1.0, "origin": [0.0, 0.0, 0.0]}
    destination = {"resolution": 1.0, "origin": [-2.0, -3.0, 0.0]}
    affine = CORE.map_affine(source, 10, destination, 20)
    actual = affine @ np.array([3.0, 4.0, 1.0])
    assert np.allclose(actual, [5.0, 11.0])


def test_merge_multiple_classes(tmp_path):
    base_grid = np.full((6, 6), 254, np.uint8)
    base_index = np.full((6, 6), 2, np.uint8)
    base = write_map(tmp_path / "base", "base", base_grid, base_index)
    patch_grid = np.full((3, 3), 254, np.uint8)
    patch_index = np.full((3, 3), 2, np.uint8)
    patch_index[0, 0] = 3
    patch_index[2, 2] = 4
    patch = write_map(
        tmp_path / "patch", "patch", patch_grid, patch_index, origin=(1, 1)
    )

    result = CORE.merge_semantic_classes_patch(
        base.parent,
        patch.parent,
        {"grass": 0.5, "tactile paving": 0.4},
        "/tmp/bag",
        10,
        20,
        tmp_path / "result",
    )
    merged_base = CORE.find_map_base(result.output_directory)
    merged = cv2.imread(
        str(merged_base.with_name(merged_base.name + ".colored.pgm")), 0
    )
    assert merged[2, 1] == 3
    assert merged[4, 3] == 4
    assert result.added_cells_by_class == {"grass": 1, "tactile paving": 1}
