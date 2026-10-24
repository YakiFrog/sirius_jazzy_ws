#!/usr/bin/env python3
"""Print the canonical Nav2 mode list for the launcher UI."""
import json
import sys
from pathlib import Path

try:
    from sirius_navigation.navigation_modes import (
        NAVIGATION_MODE_CONFIGS,
        NAVIGATION_MODE_INFO,
        NAVIGATION_MODE_PRESENTATION,
    )
except ImportError:
    workspace = Path(__file__).resolve().parents[2]
    sys.path.insert(0, str(workspace / "src/sirius/sirius_navigation"))
    from sirius_navigation.navigation_modes import (  # noqa: E402
        NAVIGATION_MODE_CONFIGS,
        NAVIGATION_MODE_INFO,
        NAVIGATION_MODE_PRESENTATION,
    )


def main():
    modes = []
    for mode, presentation in NAVIGATION_MODE_PRESENTATION.items():
        if mode not in NAVIGATION_MODE_CONFIGS or mode not in NAVIGATION_MODE_INFO:
            continue
        info = NAVIGATION_MODE_INFO[mode]
        modes.append({
            "mode": mode,
            "label": presentation["label"],
            "description": presentation["description"],
            "speed": float(info["speed"]),
        })
    print(json.dumps({"modes": modes}, ensure_ascii=False))


if __name__ == "__main__":
    main()
