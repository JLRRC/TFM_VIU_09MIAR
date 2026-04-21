#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_tools/ur5_tools/gripper_geometry.py
# Contenido: Geometria canonica del gripper RG2 derivada del URDF del robot.
"""Resuelve la geometria canonica de agarre del RG2 desde el URDF fuente de verdad."""

from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache
import math
import os
from pathlib import Path
import re
from typing import Iterable, Optional, Tuple

try:
    from ament_index_python.packages import get_package_share_directory
except Exception:  # pragma: no cover - ament may be unavailable in unit contexts
    get_package_share_directory = None


TOOL0_FRAME = "tool0"
RG2_TCP_FRAME = "rg2_tcp"
RG2_PINCH_CENTER_FRAME = "rg2_pinch_center"
RG2_TCP_JOINT = "rg2_tcp_joint"
RG2_PINCH_CENTER_JOINT = "rg2_pinch_center_joint"
PICK_DEMO_ANCHOR_JOINT = "pick_demo_anchor_joint"


@dataclass(frozen=True)
class FixedJointOrigin:
    """Origen de una articulacion fija leido del URDF canonico."""

    parent_link: str
    child_link: str
    xyz: Tuple[float, float, float]
    source_path: str


@dataclass(frozen=True)
class GripperGeometry:
    """Geometria canonica de agarre del RG2 resuelta desde el URDF."""

    tcp: FixedJointOrigin
    pinch_center: FixedJointOrigin

    def origin_for_frame(self, frame_name: str) -> FixedJointOrigin:
        frame = str(frame_name or "").strip()
        if frame == RG2_TCP_FRAME:
            return self.tcp
        if frame == RG2_PINCH_CENTER_FRAME:
            return self.pinch_center
        raise KeyError(f"unsupported gripper frame: {frame_name!r}")

    def xyz_for_frame(self, frame_name: str) -> Tuple[float, float, float]:
        return self.origin_for_frame(frame_name).xyz

    def z_for_frame(self, frame_name: str) -> float:
        return float(self.xyz_for_frame(frame_name)[2])


def _candidate_urdf_paths(ws_dir: Optional[str] = None) -> Iterable[Path]:
    seen: set[Path] = set()

    def _push(path: Path) -> Iterable[Path]:
        resolved = path.expanduser().resolve()
        if resolved in seen:
            return ()
        seen.add(resolved)
        return (resolved,)

    env_path = str(os.environ.get("UR5_GEOMETRY_URDF_XACRO", "") or "").strip()
    if env_path:
        yield from _push(Path(env_path))

    ws_dir_value = str(ws_dir or os.environ.get("WS_DIR", "") or "").strip()
    if ws_dir_value:
        yield from _push(
            Path(ws_dir_value) / "src" / "ur5_description" / "urdf" / "ur5.urdf.xacro"
        )

    # Fallback al arbol fuente para tests/imports locales antes de que exista install-space.
    try:
        workspace_root = Path(__file__).resolve().parents[3]
    except Exception:  # pragma: no cover - defensive
        workspace_root = None
    if workspace_root is not None:
        yield from _push(
            workspace_root / "src" / "ur5_description" / "urdf" / "ur5.urdf.xacro"
        )

    if get_package_share_directory is not None:
        try:
            share_dir = Path(get_package_share_directory("ur5_description"))
        except Exception:
            share_dir = None
        if share_dir is not None:
            yield from _push(share_dir / "urdf" / "ur5.urdf.xacro")


def _read_text(path: Path) -> str:
    with path.open("r", encoding="utf-8") as handle:
        return handle.read()


def _parse_xyz(raw_xyz: str, *, source_path: str, joint_name: str) -> Tuple[float, float, float]:
    parts = [token for token in str(raw_xyz or "").strip().split() if token]
    if len(parts) != 3:
        raise ValueError(
            f"invalid xyz for {joint_name} in {source_path}: expected 3 values, got {raw_xyz!r}"
        )
    return (float(parts[0]), float(parts[1]), float(parts[2]))


def _parse_joint_origin(urdf_text: str, *, joint_name: str, source_path: str) -> FixedJointOrigin:
    pattern = re.compile(
        rf'<joint name="{re.escape(joint_name)}"[^>]*>'
        rf".*?<parent link=\"([^\"]+)\"/>"
        rf".*?<child link=\"([^\"]+)\"/>"
        rf'.*?<origin xyz="([^"]+)" rpy="[^"]*"/>'
        rf".*?</joint>",
        re.DOTALL,
    )
    match = pattern.search(urdf_text)
    if not match:
        raise ValueError(f"joint {joint_name} not found in {source_path}")
    parent_link, child_link, xyz_raw = match.groups()
    return FixedJointOrigin(
        parent_link=str(parent_link).strip(),
        child_link=str(child_link).strip(),
        xyz=_parse_xyz(xyz_raw, source_path=source_path, joint_name=joint_name),
        source_path=source_path,
    )


@lru_cache(maxsize=4)
def load_gripper_geometry(ws_dir: Optional[str] = None) -> GripperGeometry:
    """Carga del URDF los offsets canonicos del TCP y del pinch-center del RG2."""

    for path in _candidate_urdf_paths(ws_dir):
        if not path.is_file():
            continue
        urdf_text = _read_text(path)
        source_path = str(path)
        tcp = _parse_joint_origin(
            urdf_text,
            joint_name=RG2_TCP_JOINT,
            source_path=source_path,
        )
        pinch_center = _parse_joint_origin(
            urdf_text,
            joint_name=RG2_PINCH_CENTER_JOINT,
            source_path=source_path,
        )
        return GripperGeometry(tcp=tcp, pinch_center=pinch_center)
    raise FileNotFoundError(
        "unable to locate ur5.urdf.xacro for canonical RG2 geometry resolution"
    )


def format_xyz(xyz: Tuple[float, float, float], *, digits: int = 7) -> str:
    """Formatea una tupla XYZ para logs y para parchear el SDF."""

    return " ".join(f"{float(value):.{digits}f}" for value in xyz)


def vector_distance(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> float:
    """Devuelve la distancia euclidea entre dos tuplas XYZ."""

    return math.sqrt(
        ((float(a[0]) - float(b[0])) ** 2)
        + ((float(a[1]) - float(b[1])) ** 2)
        + ((float(a[2]) - float(b[2])) ** 2)
    )


def patch_runtime_model_sdf(
    model_sdf_path: str,
    *,
    controllers_yaml: Optional[str] = None,
    geometry: Optional[GripperGeometry] = None,
) -> str:
    """Parchea el SDF de runtime para que Gazebo use la geometria canonica."""

    geometry = geometry or load_gripper_geometry()
    path = Path(model_sdf_path).expanduser().resolve()
    text = _read_text(path)

    if controllers_yaml:
        plugin_re = re.compile(
            r'(<plugin filename="gz_ros2_control-system"[^>]*>)(.*?)(</plugin>)',
            re.DOTALL,
        )
        match = plugin_re.search(text)
        if match:
            header, body, footer = match.groups()
            if "<parameters>" in body:
                body = re.sub(
                    r"<parameters>.*?</parameters>",
                    f"<parameters>{controllers_yaml}</parameters>",
                    body,
                    flags=re.DOTALL,
                )
            else:
                body = body + f"\n            <parameters>{controllers_yaml}</parameters>\n"
            text = text[: match.start()] + header + body + footer + text[match.end() :]

    anchor_xyz = geometry.xyz_for_frame(RG2_TCP_FRAME)
    joint_re = re.compile(
        r'(<joint name="pick_demo_anchor_joint" type="fixed">)(.*?)(</joint>)',
        re.DOTALL,
    )
    match = joint_re.search(text)
    if not match:
        raise ValueError(f"joint {PICK_DEMO_ANCHOR_JOINT} not found in {path}")
    header, body, footer = match.groups()
    if 'relative_to="tool0"' not in body:
        raise ValueError(f"{PICK_DEMO_ANCHOR_JOINT} in {path} is not relative_to=\"tool0\"")
    body = re.sub(
        r'<pose relative_to="tool0">.*?</pose>',
        f'<pose relative_to="tool0">{format_xyz(anchor_xyz)} 0 0 0</pose>',
        body,
        count=1,
    )
    text = text[: match.start()] + header + body + footer + text[match.end() :]
    path.write_text(text, encoding="utf-8")
    return str(path)


def read_pick_demo_anchor_xyz(model_sdf_path: str) -> Tuple[float, float, float]:
    """Lee del SDF la pose de `pick_demo_anchor_joint` relativa a `tool0`."""

    path = Path(model_sdf_path).expanduser().resolve()
    text = _read_text(path)
    pattern = re.compile(
        r'<joint name="pick_demo_anchor_joint" type="fixed">'
        r'.*?<pose relative_to="tool0">([^<]+)</pose>'
        r".*?</joint>",
        re.DOTALL,
    )
    match = pattern.search(text)
    if not match:
        raise ValueError(f"joint {PICK_DEMO_ANCHOR_JOINT} not found in {path}")
    values = [token for token in match.group(1).strip().split() if token]
    if len(values) < 3:
        raise ValueError(f"invalid pose for {PICK_DEMO_ANCHOR_JOINT} in {path}")
    return (float(values[0]), float(values[1]), float(values[2]))


def validate_pick_demo_anchor(
    model_sdf_path: str,
    *,
    geometry: Optional[GripperGeometry] = None,
    tolerance_m: float = 1e-6,
) -> Tuple[bool, str]:
    """Comprueba que el anclaje desmontable de Gazebo coincide con el TCP canonico."""

    geometry = geometry or load_gripper_geometry()
    expected_xyz = geometry.xyz_for_frame(RG2_TCP_FRAME)
    anchor_xyz = read_pick_demo_anchor_xyz(model_sdf_path)
    error_m = vector_distance(anchor_xyz, expected_xyz)
    if error_m > float(tolerance_m):
        return (
            False,
            "pick_demo_anchor_joint mismatch "
            f"actual=({format_xyz(anchor_xyz)}) expected=({format_xyz(expected_xyz)}) "
            f"err_m={error_m:.6f}",
        )
    return True, f"pick_demo_anchor_joint ok err_m={error_m:.6f}"
