#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/scripts/sync_object_positions_to_sdf.py
# Contenido: Script operativo o de diagnostico del workspace ROS 2.
# Uso breve: Se ejecuta manualmente para arrancar, medir o validar el stack.
"""Sync object positions from JSON into an SDF world file."""
from __future__ import annotations

import argparse
import json
import os
import sys
import xml.etree.ElementTree as ET
from typing import Dict, Tuple


def _load_positions(path: str) -> Dict[str, Tuple[float, float, float]]:
    with open(path, "r", encoding="utf-8") as f:
        raw = json.load(f)
    if not isinstance(raw, dict):
        raise ValueError("object_positions.json must be a dict of name -> [x,y,z]")
    parsed: Dict[str, Tuple[float, float, float]] = {}
    for name, value in raw.items():
        if not isinstance(value, (list, tuple)) or len(value) < 3:
            continue
        parsed[name] = (float(value[0]), float(value[1]), float(value[2]))
    return parsed


def _parse_pose(text: str) -> Tuple[float, float, float, float, float, float]:
    parts = [float(v) for v in text.split()]
    while len(parts) < 6:
        parts.append(0.0)
    return tuple(parts[:6])


def _format_pose(values: Tuple[float, float, float, float, float, float]) -> str:
    return f"{values[0]:.3f} {values[1]:.3f} {values[2]:.3f} {values[3]:.3f} {values[4]:.3f} {values[5]:.3f}"


def sync_positions(sdf_path: str, json_path: str) -> int:
    positions = _load_positions(json_path)
    tree = ET.parse(sdf_path)
    root = tree.getroot()
    changed = 0
    for model in root.iter("model"):
        name = model.attrib.get("name", "")
        if name not in positions:
            continue
        pose_elem = model.find("pose")
        if pose_elem is None or not pose_elem.text:
            pose_elem = ET.SubElement(model, "pose")
            pose = (positions[name][0], positions[name][1], positions[name][2], 0.0, 0.0, 0.0)
        else:
            pose = _parse_pose(pose_elem.text)
            pose = (positions[name][0], positions[name][1], positions[name][2], pose[3], pose[4], pose[5])
        pose_elem.text = _format_pose(pose)
        changed += 1
    if changed == 0:
        print("[sync] no matching models found", file=sys.stderr)
        return 1
    tree.write(sdf_path, encoding="utf-8", xml_declaration=True)
    print(f"[sync] updated {changed} models in {sdf_path}")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Sync object positions into SDF.")
    parser.add_argument("--sdf", required=True, help="Path to SDF world file.")
    parser.add_argument("--json", required=True, help="Path to object_positions.json.")
    args = parser.parse_args()
    sdf_path = os.path.expanduser(args.sdf)
    json_path = os.path.expanduser(args.json)
    if not os.path.isfile(sdf_path):
        print(f"[sync] SDF not found: {sdf_path}", file=sys.stderr)
        return 2
    if not os.path.isfile(json_path):
        print(f"[sync] JSON not found: {json_path}", file=sys.stderr)
        return 2
    return sync_positions(sdf_path, json_path)


if __name__ == "__main__":
    raise SystemExit(main())
