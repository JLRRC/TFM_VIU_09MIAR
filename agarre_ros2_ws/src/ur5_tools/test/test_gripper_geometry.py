#!/usr/bin/env python3
"""Tests unitarios de los helpers de geometria canonica del RG2."""

from __future__ import annotations

from pathlib import Path

import pytest

import ur5_tools.gripper_geometry as _gg

from ur5_tools.gripper_geometry import (
    TOOL0_FRAME,
    _candidate_urdf_paths,
    _parse_joint_origin,
    _parse_xyz,
    _resolve_xyz_value,
    contact_z_correction_for_frame,
    evaluate_geometry_snapshot,
    evaluate_runtime_geometry,
    format_xyz,
    load_gripper_geometry,
    patch_runtime_model_sdf,
    RG2_PINCH_CENTER_FRAME,
    RG2_TCP_FRAME,
    read_pick_demo_anchor_xyz,
    tool0_offset_for_frame,
    validate_pick_demo_anchor,
    vector_distance,
)


ROOT = Path(__file__).resolve().parents[3]


def test_load_gripper_geometry_matches_validated_tcp_fix() -> None:
    geometry = load_gripper_geometry(str(ROOT))
    tcp_xyz = geometry.xyz_for_frame(RG2_TCP_FRAME)
    pinch_xyz = geometry.xyz_for_frame(RG2_PINCH_CENTER_FRAME)
    assert tcp_xyz[2] == pinch_xyz[2]
    assert tcp_xyz[2] == 0.175
    assert vector_distance(tcp_xyz, pinch_xyz) == 0.0


def test_model_anchor_matches_canonical_geometry() -> None:
    ok, reason = validate_pick_demo_anchor(
        # F5 audit (2026-05-10): models en src/ur5_gazebo/models.
        str(ROOT / 'src' / 'ur5_gazebo' / 'models' / 'ur5_rg2' / 'model.sdf'),
        geometry=load_gripper_geometry(str(ROOT)),
        tolerance_m=1e-6,
    )
    assert ok, reason


def test_contact_z_correction_is_urdf_driven() -> None:
    geometry = load_gripper_geometry(str(ROOT))
    assert contact_z_correction_for_frame(RG2_PINCH_CENTER_FRAME, geometry=geometry) == 0.0
    assert contact_z_correction_for_frame(RG2_TCP_FRAME, geometry=geometry) == 0.0
    assert contact_z_correction_for_frame(TOOL0_FRAME, geometry=geometry) == 0.175


def test_tool0_offset_for_contact_frames_is_urdf_driven() -> None:
    geometry = load_gripper_geometry(str(ROOT))
    assert tool0_offset_for_frame(RG2_PINCH_CENTER_FRAME, geometry=geometry) == (
        0.0,
        0.0,
        0.175,
    )
    assert tool0_offset_for_frame(RG2_TCP_FRAME, geometry=geometry) == (
        0.0,
        0.0,
        0.175,
    )


def test_evaluate_geometry_snapshot_accepts_matching_runtime_capture() -> None:
    geometry = load_gripper_geometry(str(ROOT))
    ok, reason, snapshot = evaluate_geometry_snapshot(
        {
            RG2_TCP_FRAME: geometry.xyz_for_frame(RG2_TCP_FRAME),
            RG2_PINCH_CENTER_FRAME: geometry.xyz_for_frame(RG2_PINCH_CENTER_FRAME),
        },
        geometry=geometry,
    )
    assert ok, reason
    assert reason == "ok"
    assert snapshot["pair_error_m"] == 0.0
    assert snapshot["source_path"] == geometry.tcp.source_path


def test_evaluate_geometry_snapshot_rejects_runtime_mismatch() -> None:
    geometry = load_gripper_geometry(str(ROOT))
    ok, reason, snapshot = evaluate_geometry_snapshot(
        {
            RG2_TCP_FRAME: (0.0, 0.0, 0.070),
            RG2_PINCH_CENTER_FRAME: geometry.xyz_for_frame(RG2_PINCH_CENTER_FRAME),
        },
        geometry=geometry,
        offset_tol_m=0.002,
    )
    assert not ok
    assert "tool0->rg2_tcp err_m=" in reason
    assert snapshot["frame_error_m"][RG2_TCP_FRAME] > 0.09


def test_evaluate_runtime_geometry_uses_single_runtime_lookup_callback() -> None:
    geometry = load_gripper_geometry(str(ROOT))

    def _lookup(tool_frame: str, frame_name: str):
        assert tool_frame == TOOL0_FRAME
        return geometry.xyz_for_frame(frame_name)

    ok, reason, snapshot = evaluate_runtime_geometry(
        _lookup,
        geometry=geometry,
        tool_frame=TOOL0_FRAME,
    )

    assert ok, reason
    assert reason == "ok"
    assert snapshot["ok"] is True
    assert snapshot["reason"] == "ok"
    assert snapshot["urdf_source"] == geometry.tcp.source_path


def test_evaluate_runtime_geometry_returns_partial_snapshot_on_lookup_failure() -> None:
    geometry = load_gripper_geometry(str(ROOT))

    def _lookup(tool_frame: str, frame_name: str):
        if frame_name == RG2_TCP_FRAME:
            return geometry.xyz_for_frame(frame_name)
        raise LookupError(f"{tool_frame}->{frame_name} missing")

    ok, reason, snapshot = evaluate_runtime_geometry(
        _lookup,
        geometry=geometry,
        tool_frame=TOOL0_FRAME,
    )

    assert not ok
    assert reason == "tool0->rg2_pinch_center missing"
    assert snapshot["ok"] is False
    assert snapshot["reason"] == "tool0->rg2_pinch_center missing"
    assert snapshot["actual_xyz"][RG2_TCP_FRAME] == list(geometry.xyz_for_frame(RG2_TCP_FRAME))


def test_format_xyz_default_digits() -> None:
    s = format_xyz((0.1, -0.2, 0.0050885))
    parts = s.split()
    assert len(parts) == 3
    assert parts[0] == "0.1000000"
    assert parts[1] == "-0.2000000"
    assert parts[2] == "0.0050885"


def test_format_xyz_custom_digits() -> None:
    s = format_xyz((1.0, 2.0, 3.0), digits=3)
    assert s == "1.000 2.000 3.000"


def test_vector_distance_zero() -> None:
    assert vector_distance((1.0, 2.0, 3.0), (1.0, 2.0, 3.0)) == 0.0


def test_vector_distance_unit() -> None:
    import math
    assert vector_distance((0.0, 0.0, 0.0), (1.0, 0.0, 0.0)) == pytest.approx(1.0)
    assert vector_distance((0.0, 0.0, 0.0), (0.0, 0.0, 1.0)) == pytest.approx(1.0)
    assert vector_distance((0.0, 0.0, 0.0), (1.0, 1.0, 1.0)) == pytest.approx(math.sqrt(3.0))


# ---------------------------------------------------------------------------
# Line 54: origin_for_frame — unsupported frame
# ---------------------------------------------------------------------------

def test_origin_for_frame_unknown_raises_key_error() -> None:
    geometry = load_gripper_geometry(str(ROOT))
    with pytest.raises(KeyError, match="unsupported gripper frame"):
        geometry.origin_for_frame("unknown_frame")


# ---------------------------------------------------------------------------
# Line 110: _parse_xyz — too few values
# ---------------------------------------------------------------------------

def test_parse_xyz_too_few_values_raises() -> None:
    with pytest.raises(ValueError, match="expected 3 values"):
        _parse_xyz("0.1 0.2", source_path="t.urdf", joint_name="j")


# ---------------------------------------------------------------------------
# Lines 135, 138: _resolve_xyz_value — no-match and empty property-name paths
# ---------------------------------------------------------------------------

def test_resolve_xyz_value_plain_string_returns_as_is() -> None:
    # no ${...} pattern → if not match: return value (line 135)
    assert _resolve_xyz_value("0.0 0.0 0.175") == "0.0 0.0 0.175"


def test_resolve_xyz_value_empty_property_name_returns_original() -> None:
    # "${ }" matches but property_name=" ".strip()="" → if not property_name: return (line 138)
    assert _resolve_xyz_value("${ }") == "${ }"


# ---------------------------------------------------------------------------
# Line 160: _parse_joint_origin — joint not found in text
# ---------------------------------------------------------------------------

def test_parse_joint_origin_missing_joint_raises() -> None:
    with pytest.raises(ValueError, match="not found"):
        _parse_joint_origin("no joints here", joint_name="missing", source_path="t.urdf")


# ---------------------------------------------------------------------------
# Lines 180 + 197: load_gripper_geometry — nonexistent paths → continue, FileNotFoundError
# ---------------------------------------------------------------------------

def test_load_gripper_geometry_raises_when_no_urdf_found(monkeypatch) -> None:
    def _fake_paths(ws_dir=None):
        yield Path("/nonexistent_xz9q/ur5.urdf.xacro")

    monkeypatch.setattr(_gg, "_candidate_urdf_paths", _fake_paths)
    _gg.load_gripper_geometry.cache_clear()
    with pytest.raises(FileNotFoundError):
        _gg.load_gripper_geometry("__no_urdf_test__")


# ---------------------------------------------------------------------------
# Lines 75 + 69 + 84-91: env-var path deduplicates with workspace fallback
# ---------------------------------------------------------------------------

def test_candidate_urdf_paths_env_var_deduplicates_workspace_fallback(monkeypatch) -> None:
    workspace_root = Path(_gg.__file__).resolve().parents[3]
    urdf_path = str(workspace_root / "src" / "ur5_description" / "urdf" / "ur5.urdf.xacro")
    monkeypatch.setenv("UR5_GEOMETRY_URDF_XACRO", urdf_path)
    paths = list(_candidate_urdf_paths(None))
    resolved = Path(urdf_path).resolve()
    # env-var path yields first; workspace fallback resolves to same → dedup → exactly 1
    assert resolved in paths
    assert paths.count(resolved) == 1


# ---------------------------------------------------------------------------
# Lines 93-99: ament_index fallback — success path and raising path
# ---------------------------------------------------------------------------

def test_candidate_urdf_paths_ament_index_success(monkeypatch, tmp_path) -> None:
    def _fake_get_pkg_share(pkg_name: str) -> str:
        return str(tmp_path)

    monkeypatch.setattr(_gg, "get_package_share_directory", _fake_get_pkg_share)
    paths = list(_candidate_urdf_paths("__nonexistent_ws_ament__"))
    expected = (tmp_path / "urdf" / "ur5.urdf.xacro").resolve()
    assert expected in paths


def test_candidate_urdf_paths_ament_index_raises_is_handled(monkeypatch) -> None:
    def _raising(pkg_name: str):
        raise RuntimeError("not installed")

    monkeypatch.setattr(_gg, "get_package_share_directory", _raising)
    # Should not propagate — share_dir set to None, ament path skipped
    paths = list(_candidate_urdf_paths("__nonexistent_ws_ament2__"))
    assert isinstance(paths, list)


# ---------------------------------------------------------------------------
# Line 236: contact_z_correction_for_frame — unknown frame → 0.0
# ---------------------------------------------------------------------------

def test_contact_z_correction_unknown_frame_returns_zero() -> None:
    geometry = load_gripper_geometry(str(ROOT))
    assert contact_z_correction_for_frame("some_other_frame", geometry=geometry) == 0.0


# ---------------------------------------------------------------------------
# Lines 248 + 252: tool0_offset_for_frame — unknown frame raises KeyError
# ---------------------------------------------------------------------------

def test_tool0_offset_for_tool0_returns_zero() -> None:
    geometry = load_gripper_geometry(str(ROOT))
    assert tool0_offset_for_frame(TOOL0_FRAME, geometry=geometry) == (0.0, 0.0, 0.0)


def test_tool0_offset_for_frame_unknown_raises() -> None:
    geometry = load_gripper_geometry(str(ROOT))
    with pytest.raises(KeyError, match="unsupported tool0 offset frame"):
        tool0_offset_for_frame("unknown_frame", geometry=geometry)


# ---------------------------------------------------------------------------
# Lines 323-324: evaluate_runtime_geometry — lookup returns None
# ---------------------------------------------------------------------------

def test_evaluate_runtime_geometry_none_xyz_returns_false() -> None:
    geometry = load_gripper_geometry(str(ROOT))

    def _lookup(tool_frame: str, frame_name: str):
        return None

    ok, reason, _ = evaluate_runtime_geometry(_lookup, geometry=geometry)
    assert not ok
    assert "missing" in reason


# ---------------------------------------------------------------------------
# Lines 343-345: evaluate_runtime_geometry — invalid xyz floats
# ---------------------------------------------------------------------------

def test_evaluate_runtime_geometry_invalid_xyz_returns_false() -> None:
    geometry = load_gripper_geometry(str(ROOT))

    def _lookup(tool_frame: str, frame_name: str):
        if frame_name == RG2_TCP_FRAME:
            return geometry.xyz_for_frame(frame_name)
        return ("not", "a", "float")

    ok, reason, _ = evaluate_runtime_geometry(_lookup, geometry=geometry)
    assert not ok
    assert "invalid_xyz" in reason


# ---------------------------------------------------------------------------
# Line 406: evaluate_geometry_snapshot — frame missing from actual_xyz
# ---------------------------------------------------------------------------

def test_evaluate_geometry_snapshot_missing_frame_returns_false() -> None:
    geometry = load_gripper_geometry(str(ROOT))
    ok, reason, _ = evaluate_geometry_snapshot({}, geometry=geometry)
    assert not ok
    assert "missing" in reason


# ---------------------------------------------------------------------------
# Line 428: evaluate_geometry_snapshot — pair error exceeds pair_tol_m
# ---------------------------------------------------------------------------

def test_evaluate_geometry_snapshot_pair_error_exceeds_tolerance() -> None:
    geometry = load_gripper_geometry(str(ROOT))
    # Both frames within offset_tol but separated enough to fail pair check
    ok, reason, snap = evaluate_geometry_snapshot(
        {
            RG2_TCP_FRAME: (0.001, 0.0, 0.175),
            RG2_PINCH_CENTER_FRAME: (-0.001, 0.0, 0.175),
        },
        geometry=geometry,
        offset_tol_m=0.005,
        pair_tol_m=0.001,
    )
    assert not ok
    assert "rg2_tcp_vs_rg2_pinch_center" in reason
    assert snap["pair_error_m"] > 0.001


# ---------------------------------------------------------------------------
# Lines 446-488: patch_runtime_model_sdf
# Lines 491-508: read_pick_demo_anchor_xyz
# Lines 511-530: validate_pick_demo_anchor
# ---------------------------------------------------------------------------

_ANCHOR_SDF = """\
<?xml version="1.0"?>
<sdf version="1.7">
  <model name="ur5_rg2">
    <plugin filename="gz_ros2_control-system" name="ctrl">
      <parameters>/old/controllers.yaml</parameters>
    </plugin>
    <joint name="pick_demo_anchor_joint" type="fixed">
      <parent>tool0</parent>
      <child>pick_demo_anchor</child>
      <pose relative_to="tool0">0.0000000 0.0000000 0.0050885 0 0 0</pose>
    </joint>
  </model>
</sdf>
"""

_ANCHOR_SDF_NO_PARAMS = """\
<?xml version="1.0"?>
<sdf version="1.7">
  <model name="ur5_rg2">
    <plugin filename="gz_ros2_control-system" name="ctrl">
    </plugin>
    <joint name="pick_demo_anchor_joint" type="fixed">
      <parent>tool0</parent>
      <child>pick_demo_anchor</child>
      <pose relative_to="tool0">0.0000000 0.0000000 0.0050885 0 0 0</pose>
    </joint>
  </model>
</sdf>
"""


def test_patch_runtime_model_sdf_updates_anchor_and_controllers(tmp_path) -> None:
    geometry = load_gripper_geometry(str(ROOT))
    sdf_file = tmp_path / "model.sdf"
    sdf_file.write_text(_ANCHOR_SDF, encoding="utf-8")
    result = patch_runtime_model_sdf(
        str(sdf_file), controllers_yaml="/new/controllers.yaml", geometry=geometry
    )
    assert result == str(sdf_file)
    updated = sdf_file.read_text(encoding="utf-8")
    assert "/new/controllers.yaml" in updated
    anchor_xyz = read_pick_demo_anchor_xyz(str(sdf_file))
    assert vector_distance(anchor_xyz, geometry.xyz_for_frame(RG2_TCP_FRAME)) < 1e-6


def test_patch_runtime_model_sdf_adds_parameters_when_absent(tmp_path) -> None:
    geometry = load_gripper_geometry(str(ROOT))
    sdf_file = tmp_path / "model_np.sdf"
    sdf_file.write_text(_ANCHOR_SDF_NO_PARAMS, encoding="utf-8")
    patch_runtime_model_sdf(str(sdf_file), controllers_yaml="/new/ctrl.yaml", geometry=geometry)
    assert "/new/ctrl.yaml" in sdf_file.read_text(encoding="utf-8")


def test_patch_runtime_model_sdf_without_controllers_patches_anchor(tmp_path) -> None:
    geometry = load_gripper_geometry(str(ROOT))
    sdf_file = tmp_path / "model_nc.sdf"
    sdf_file.write_text(_ANCHOR_SDF, encoding="utf-8")
    patch_runtime_model_sdf(str(sdf_file), geometry=geometry)
    anchor_xyz = read_pick_demo_anchor_xyz(str(sdf_file))
    assert vector_distance(anchor_xyz, geometry.xyz_for_frame(RG2_TCP_FRAME)) < 1e-6


def test_patch_runtime_model_sdf_raises_when_anchor_joint_missing(tmp_path) -> None:
    geometry = load_gripper_geometry(str(ROOT))
    sdf_file = tmp_path / "bad.sdf"
    sdf_file.write_text("<sdf><model></model></sdf>", encoding="utf-8")
    with pytest.raises(ValueError, match="pick_demo_anchor_joint"):
        patch_runtime_model_sdf(str(sdf_file), geometry=geometry)


def test_patch_runtime_model_sdf_raises_when_wrong_relative_to(tmp_path) -> None:
    # Joint found but pose lacks relative_to="tool0" → line 479
    geometry = load_gripper_geometry(str(ROOT))
    bad_sdf = (
        '<?xml version="1.0"?>\n<sdf><model>\n'
        '  <joint name="pick_demo_anchor_joint" type="fixed">\n'
        '    <pose>0 0 0.175 0 0 0</pose>\n'
        '  </joint>\n'
        '</model></sdf>\n'
    )
    sdf_file = tmp_path / "wrong_rel.sdf"
    sdf_file.write_text(bad_sdf, encoding="utf-8")
    with pytest.raises(ValueError, match='relative_to'):
        patch_runtime_model_sdf(str(sdf_file), geometry=geometry)


def test_read_pick_demo_anchor_xyz_raises_when_joint_missing(tmp_path) -> None:
    sdf_file = tmp_path / "no_joint.sdf"
    sdf_file.write_text("<sdf><model></model></sdf>", encoding="utf-8")
    with pytest.raises(ValueError, match="pick_demo_anchor_joint"):
        read_pick_demo_anchor_xyz(str(sdf_file))


def test_read_pick_demo_anchor_xyz_raises_when_pose_has_too_few_values(tmp_path) -> None:
    bad_sdf = (
        '<sdf><model>\n'
        '  <joint name="pick_demo_anchor_joint" type="fixed">\n'
        '    <pose relative_to="tool0">0.1 0.2</pose>\n'
        '  </joint>\n'
        '</model></sdf>\n'
    )
    sdf_file = tmp_path / "short.sdf"
    sdf_file.write_text(bad_sdf, encoding="utf-8")
    with pytest.raises(ValueError, match="invalid pose"):
        read_pick_demo_anchor_xyz(str(sdf_file))


def test_validate_pick_demo_anchor_fails_when_xyz_mismatches(tmp_path) -> None:
    geometry = load_gripper_geometry(str(ROOT))
    bad_sdf = (
        '<?xml version="1.0"?>\n<sdf><model>\n'
        '  <joint name="pick_demo_anchor_joint" type="fixed">\n'
        '    <pose relative_to="tool0">0.9990000 0.0000000 0.0000000 0 0 0</pose>\n'
        '  </joint>\n'
        '</model></sdf>\n'
    )
    sdf_file = tmp_path / "mismatch.sdf"
    sdf_file.write_text(bad_sdf, encoding="utf-8")
    ok, reason = validate_pick_demo_anchor(str(sdf_file), geometry=geometry, tolerance_m=1e-6)
    assert not ok
    assert "mismatch" in reason
