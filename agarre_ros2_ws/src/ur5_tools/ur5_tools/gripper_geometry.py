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
from typing import Callable, Dict, Iterable, Optional, Tuple

from .moveit_bridge_utils import bridge_env_str

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
CONTACT_SEMANTIC_FRAMES = frozenset((RG2_TCP_FRAME, RG2_PINCH_CENTER_FRAME))


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

    env_path = bridge_env_str("UR5_GEOMETRY_URDF_XACRO", "").strip()
    if env_path:
        yield from _push(Path(env_path))

    ws_dir_value = str(ws_dir or bridge_env_str("WS_DIR", "") or "").strip()
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


def _parse_xacro_properties(urdf_text: str) -> Dict[str, str]:
    pattern = re.compile(
        r"<xacro:property\s+name=\"([^\"]+)\"\s+value=\"([^\"]+)\"\s*/?>"
    )
    return {
        str(name).strip(): str(value).strip()
        for name, value in pattern.findall(urdf_text)
        if str(name).strip()
    }


def _resolve_xyz_value(
    raw_xyz: str,
    *,
    properties: Optional[Dict[str, str]] = None,
) -> str:
    value = str(raw_xyz or "").strip()
    match = re.fullmatch(r"\$\{([^}]+)\}", value)
    if not match:
        return value
    property_name = str(match.group(1) or "").strip()
    if not property_name:
        return value
    resolved = str((properties or {}).get(property_name, "") or "").strip()
    return resolved or value


def _parse_joint_origin(
    urdf_text: str,
    *,
    joint_name: str,
    source_path: str,
    properties: Optional[Dict[str, str]] = None,
) -> FixedJointOrigin:
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
        xyz=_parse_xyz(
            _resolve_xyz_value(xyz_raw, properties=properties),
            source_path=source_path,
            joint_name=joint_name,
        ),
        source_path=source_path,
    )


@lru_cache(maxsize=4)
def load_gripper_geometry(ws_dir: Optional[str] = None) -> GripperGeometry:
    """Carga del URDF los offsets canonicos del TCP y del pinch-center del RG2."""

    for path in _candidate_urdf_paths(ws_dir):
        if not path.is_file():
            continue
        urdf_text = _read_text(path)
        properties = _parse_xacro_properties(urdf_text)
        source_path = str(path)
        tcp = _parse_joint_origin(
            urdf_text,
            joint_name=RG2_TCP_JOINT,
            source_path=source_path,
            properties=properties,
        )
        pinch_center = _parse_joint_origin(
            urdf_text,
            joint_name=RG2_PINCH_CENTER_JOINT,
            source_path=source_path,
            properties=properties,
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


def contact_z_correction_for_frame(
    frame_name: str,
    *,
    geometry: Optional[GripperGeometry] = None,
) -> float:
    """Devuelve la correccion Z necesaria para convertir un frame al punto de contacto.

    `rg2_tcp` y `rg2_pinch_center` ya son frames semanticos de contacto y, por tanto,
    no requieren compensacion adicional. `tool0` si necesita la traslacion canonica
    leida del URDF.
    """

    frame = str(frame_name or "").strip()
    if not frame or frame in CONTACT_SEMANTIC_FRAMES:
        return 0.0
    geometry = geometry or load_gripper_geometry()
    if frame == TOOL0_FRAME:
        return geometry.z_for_frame(RG2_PINCH_CENTER_FRAME)
    return 0.0


def tool0_offset_for_frame(
    frame_name: str,
    *,
    geometry: Optional[GripperGeometry] = None,
) -> Tuple[float, float, float]:
    """Devuelve el offset local `tool0 -> frame_name` desde la geometría canónica."""

    frame = str(frame_name or "").strip()
    if frame == TOOL0_FRAME:
        return (0.0, 0.0, 0.0)
    geometry = geometry or load_gripper_geometry()
    if frame in CONTACT_SEMANTIC_FRAMES:
        return geometry.xyz_for_frame(frame)
    raise KeyError(f"unsupported tool0 offset frame: {frame_name!r}")


def _runtime_geometry_snapshot(
    *,
    geometry: Optional[GripperGeometry] = None,
    tool_frame: str = TOOL0_FRAME,
    offset_tol_m: float = 0.002,
    pair_tol_m: float = 0.001,
    actual_xyz: Optional[Dict[str, Tuple[float, float, float]]] = None,
    ok: bool = False,
    reason: str = "",
) -> Dict[str, object]:
    geometry = geometry or load_gripper_geometry()
    snapshot: Dict[str, object] = {
        "tool_frame": str(tool_frame or TOOL0_FRAME),
        "source_path": geometry.tcp.source_path,
        "urdf_source": geometry.tcp.source_path,
        "expected_xyz": {
            RG2_TCP_FRAME: list(geometry.xyz_for_frame(RG2_TCP_FRAME)),
            RG2_PINCH_CENTER_FRAME: list(geometry.xyz_for_frame(RG2_PINCH_CENTER_FRAME)),
        },
        "actual_xyz": {},
        "frame_error_m": {},
        "offset_tol_m": float(offset_tol_m),
        "pair_tol_m": float(pair_tol_m),
        "pair_error_m": None,
        "ok": bool(ok),
        "reason": str(reason or ""),
    }
    for frame_name, xyz in (actual_xyz or {}).items():
        snapshot["actual_xyz"][frame_name] = [
            float(xyz[0]),
            float(xyz[1]),
            float(xyz[2]),
        ]
    return snapshot


def evaluate_runtime_geometry(
    lookup_xyz_fn: Callable[[str, str], Tuple[float, float, float]],
    *,
    geometry: Optional[GripperGeometry] = None,
    tool_frame: str = TOOL0_FRAME,
    offset_tol_m: float = 0.002,
    pair_tol_m: float = 0.001,
) -> Tuple[bool, str, Dict[str, object]]:
    """Evalúa la geometría runtime usando un callback `tool_frame -> frame`."""

    geometry = geometry or load_gripper_geometry()
    normalized_tool_frame = str(tool_frame or TOOL0_FRAME)
    actual_xyz: Dict[str, Tuple[float, float, float]] = {}
    for frame_name in (RG2_TCP_FRAME, RG2_PINCH_CENTER_FRAME):
        try:
            raw_xyz = lookup_xyz_fn(normalized_tool_frame, frame_name)
        except Exception as exc:
            reason = str(exc or "").strip() or f"{normalized_tool_frame}->{frame_name} lookup_error"
            return (
                False,
                reason,
                _runtime_geometry_snapshot(
                    geometry=geometry,
                    tool_frame=normalized_tool_frame,
                    offset_tol_m=offset_tol_m,
                    pair_tol_m=pair_tol_m,
                    actual_xyz=actual_xyz,
                    ok=False,
                    reason=reason,
                ),
            )
        if raw_xyz is None:
            reason = f"{normalized_tool_frame}->{frame_name} missing"
            return (
                False,
                reason,
                _runtime_geometry_snapshot(
                    geometry=geometry,
                    tool_frame=normalized_tool_frame,
                    offset_tol_m=offset_tol_m,
                    pair_tol_m=pair_tol_m,
                    actual_xyz=actual_xyz,
                    ok=False,
                    reason=reason,
                ),
            )
        try:
            actual_xyz[frame_name] = (
                float(raw_xyz[0]),
                float(raw_xyz[1]),
                float(raw_xyz[2]),
            )
        except Exception:
            reason = f"{normalized_tool_frame}->{frame_name} invalid_xyz"
            return (
                False,
                reason,
                _runtime_geometry_snapshot(
                    geometry=geometry,
                    tool_frame=normalized_tool_frame,
                    offset_tol_m=offset_tol_m,
                    pair_tol_m=pair_tol_m,
                    actual_xyz=actual_xyz,
                    ok=False,
                    reason=reason,
                ),
            )
    ok, reason, snapshot = evaluate_geometry_snapshot(
        actual_xyz,
        geometry=geometry,
        tool_frame=normalized_tool_frame,
        offset_tol_m=offset_tol_m,
        pair_tol_m=pair_tol_m,
    )
    snapshot["ok"] = bool(ok)
    snapshot["reason"] = str(reason or "")
    snapshot["urdf_source"] = str(
        snapshot.get("urdf_source") or snapshot.get("source_path") or geometry.tcp.source_path
    )
    return ok, reason, snapshot


def evaluate_geometry_snapshot(
    actual_xyz: Dict[str, Tuple[float, float, float]],
    *,
    geometry: Optional[GripperGeometry] = None,
    tool_frame: str = TOOL0_FRAME,
    offset_tol_m: float = 0.002,
    pair_tol_m: float = 0.001,
) -> Tuple[bool, str, Dict[str, object]]:
    """Evalúa una captura runtime del gripper contra la geometría canónica."""

    geometry = geometry or load_gripper_geometry()
    expected_xyz = {
        RG2_TCP_FRAME: geometry.xyz_for_frame(RG2_TCP_FRAME),
        RG2_PINCH_CENTER_FRAME: geometry.xyz_for_frame(RG2_PINCH_CENTER_FRAME),
    }
    snapshot: Dict[str, object] = {
        "tool_frame": str(tool_frame or TOOL0_FRAME),
        "source_path": geometry.tcp.source_path,
        "expected_xyz": {
            frame_name: list(expected_xyz[frame_name])
            for frame_name in (RG2_TCP_FRAME, RG2_PINCH_CENTER_FRAME)
        },
        "actual_xyz": {},
        "frame_error_m": {},
        "offset_tol_m": float(offset_tol_m),
        "pair_tol_m": float(pair_tol_m),
        "pair_error_m": None,
    }

    normalized_actual: Dict[str, Tuple[float, float, float]] = {}
    for frame_name in (RG2_TCP_FRAME, RG2_PINCH_CENTER_FRAME):
        raw_actual = actual_xyz.get(frame_name)
        if raw_actual is None:
            return False, f"{tool_frame}->{frame_name} missing", snapshot
        actual = (float(raw_actual[0]), float(raw_actual[1]), float(raw_actual[2]))
        expected = expected_xyz[frame_name]
        err_m = vector_distance(actual, expected)
        normalized_actual[frame_name] = actual
        snapshot["actual_xyz"][frame_name] = list(actual)
        snapshot["frame_error_m"][frame_name] = float(err_m)
        if err_m > float(offset_tol_m):
            return (
                False,
                f"{tool_frame}->{frame_name} err_m={err_m:.4f} "
                f"actual=({format_xyz(actual, digits=4)}) "
                f"expected=({format_xyz(expected, digits=4)})",
                snapshot,
            )

    pair_err_m = vector_distance(
        normalized_actual[RG2_TCP_FRAME],
        normalized_actual[RG2_PINCH_CENTER_FRAME],
    )
    snapshot["pair_error_m"] = float(pair_err_m)
    if pair_err_m > float(pair_tol_m):
        return (
            False,
            "rg2_tcp_vs_rg2_pinch_center err_m="
            f"{pair_err_m:.4f} actual_tcp=({format_xyz(normalized_actual[RG2_TCP_FRAME], digits=4)}) "
            f"actual_pinch=({format_xyz(normalized_actual[RG2_PINCH_CENTER_FRAME], digits=4)})",
            snapshot,
        )
    return True, "ok", snapshot


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
