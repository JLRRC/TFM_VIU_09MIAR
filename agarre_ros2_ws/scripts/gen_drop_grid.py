#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/scripts/gen_drop_grid.py
# Contenido: Script operativo o de diagnostico del workspace ROS 2.
# Uso breve: Se ejecuta manualmente para arrancar, medir o validar el stack.
"""Generate a non-overlapping grid for drop (air) objects in object_positions.json."""
from __future__ import annotations

import argparse
import json
import os
import sys
from typing import Dict, List, Tuple

DROP_OBJECT_NAMES = [
    "box_red",
    "box_blue",
    "box_green",
    "cyl_gray",
    "cyl_orange",
    "cyl_purple",
    "box_lightblue",
    "cyl_green",
    "box_yellow",
    "cross_cyan",
]


def _load_positions(path: str) -> Dict[str, List[float]]:
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    if not isinstance(data, dict):
        raise ValueError("object_positions.json must be a dict of name -> [x,y,z]")
    return {str(k): list(v) for k, v in data.items() if isinstance(v, (list, tuple)) and len(v) >= 3}


def _save_positions(path: str, data: Dict[str, List[float]]) -> None:
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, sort_keys=False)
        f.write("\n")


def _grid_positions(
    names: List[str],
    origin: Tuple[float, float],
    cols: int,
    dx: float,
    dy: float,
    z: float,
) -> Dict[str, List[float]]:
    if cols <= 0:
        raise ValueError("cols must be >= 1")
    positions: Dict[str, List[float]] = {}
    ox, oy = origin
    for idx, name in enumerate(names):
        col = idx % cols
        row = idx // cols
        x = ox + (col * dx)
        y = oy + (row * dy)
        positions[name] = [round(x, 3), round(y, 3), round(z, 3)]
    return positions


def main() -> int:
    parser = argparse.ArgumentParser(description="Grid layout for drop (air) objects.")
    parser.add_argument("--json", required=True, help="Path to object_positions.json.")
    parser.add_argument("--z", type=float, default=2.775, help="Spawn height for drop (air) objects.")
    parser.add_argument("--cols", type=int, default=4, help="Number of columns in the grid.")
    parser.add_argument("--dx", type=float, default=0.12, help="Grid spacing in X.")
    parser.add_argument("--dy", type=float, default=-0.12, help="Grid spacing in Y (negative moves down).")
    parser.add_argument("--origin-x", type=float, default=-0.32, help="Grid origin X.")
    parser.add_argument("--origin-y", type=float, default=0.18, help="Grid origin Y.")
    parser.add_argument("--names", nargs="*", default=DROP_OBJECT_NAMES, help="Drop (air) object names.")
    args = parser.parse_args()

    json_path = os.path.expanduser(args.json)
    if not os.path.isfile(json_path):
        print(f"[grid] JSON not found: {json_path}", file=sys.stderr)
        return 2

    data = _load_positions(json_path)
    names = [str(n) for n in args.names if str(n)]
    grid = _grid_positions(
        names,
        origin=(args.origin_x, args.origin_y),
        cols=args.cols,
        dx=args.dx,
        dy=args.dy,
        z=args.z,
    )
    data.update(grid)
    _save_positions(json_path, data)
    print(f"[grid] updated {len(grid)} drop (air) objects in {json_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
