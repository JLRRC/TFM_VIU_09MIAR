#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/src/ur5_bringup/test/test_launch_helpers.py
# Contenido: Unit tests for launch_helpers.py — pure Python, no ROS needed.
"""Tests for ur5_bringup launch helpers — run without colcon or ROS."""

import time

from launch_helpers import (
    PANEL_ENV_DEFAULTS,
    build_gz_plugin_path,
    build_gz_resource_path,
    copy_runtime_model,
    patch_bridge_yaml,
    prepare_runtime_world_sdf,
    resolve_gz_partition,
    resolve_world_name,
)


# ---------------------------------------------------------------------------
# resolve_world_name
# ---------------------------------------------------------------------------

def test_resolve_world_name_finds_name(tmp_path):
    sdf = tmp_path / "world.sdf"
    sdf.write_text('<world name="my_world"><model/></world>', encoding="utf-8")
    assert resolve_world_name(str(sdf)) == "my_world"


def test_resolve_world_name_default_on_missing():
    assert resolve_world_name("/nonexistent/world.sdf") == "ur5_mesa_objetos"


def test_resolve_world_name_default_on_no_tag(tmp_path):
    sdf = tmp_path / "world.sdf"
    sdf.write_text("<root/>", encoding="utf-8")
    assert resolve_world_name(str(sdf)) == "ur5_mesa_objetos"


def test_resolve_world_name_custom_default(tmp_path):
    sdf = tmp_path / "world.sdf"
    sdf.write_text("<root/>", encoding="utf-8")
    assert resolve_world_name(str(sdf), default="fallback_world") == "fallback_world"


# ---------------------------------------------------------------------------
# resolve_gz_partition
# ---------------------------------------------------------------------------

def test_resolve_gz_partition_uses_env(tmp_path, monkeypatch):
    monkeypatch.setenv("GZ_PARTITION", "my_partition")
    result = resolve_gz_partition(str(tmp_path))
    assert result == "my_partition"


def test_resolve_gz_partition_generates_if_empty(tmp_path, monkeypatch):
    monkeypatch.delenv("GZ_PARTITION", raising=False)
    before = int(time.time())
    result = resolve_gz_partition(str(tmp_path))
    after = int(time.time())
    assert result.startswith("ur5pro_")
    ts = int(result[len("ur5pro_"):])
    assert before <= ts <= after + 1


def test_resolve_gz_partition_writes_file(tmp_path, monkeypatch):
    monkeypatch.setenv("GZ_PARTITION", "test_part")
    resolve_gz_partition(str(tmp_path))
    assert (tmp_path / "gz_partition.txt").read_text() == "test_part"


def test_resolve_gz_partition_ok_if_dir_missing(tmp_path, monkeypatch):
    monkeypatch.delenv("GZ_PARTITION", raising=False)
    result = resolve_gz_partition(str(tmp_path / "nonexistent"))
    assert result.startswith("ur5pro_")


# ---------------------------------------------------------------------------
# patch_bridge_yaml
# ---------------------------------------------------------------------------

def test_patch_bridge_yaml_replaces_world_name(tmp_path):
    base = tmp_path / "base.yaml"
    base.write_text("topic: /world/ur5_mesa_objetos/info\n", encoding="utf-8")
    runtime = tmp_path / "runtime.yaml"
    result = patch_bridge_yaml(str(base), str(runtime), "my_world")
    assert result == str(runtime)
    assert "my_world" in runtime.read_text()
    assert "ur5_mesa_objetos" not in runtime.read_text()


def test_patch_bridge_yaml_no_replacement_if_no_world_name(tmp_path):
    base = tmp_path / "base.yaml"
    base.write_text("topic: /world/ur5_mesa_objetos/info\n", encoding="utf-8")
    runtime = tmp_path / "runtime.yaml"
    patch_bridge_yaml(str(base), str(runtime), "")
    assert "ur5_mesa_objetos" in runtime.read_text()


def test_patch_bridge_yaml_fallback_on_missing_base(tmp_path):
    result = patch_bridge_yaml("/nonexistent.yaml", str(tmp_path / "r.yaml"), "w")
    assert result == "/nonexistent.yaml"


# ---------------------------------------------------------------------------
# copy_runtime_model
# ---------------------------------------------------------------------------

def test_copy_runtime_model_copies(tmp_path):
    src = tmp_path / "src_model"
    src.mkdir()
    (src / "model.sdf").write_text("<model/>")
    dst = tmp_path / "dst_model"
    copy_runtime_model(str(src), str(dst))
    assert (dst / "model.sdf").exists()


def test_copy_runtime_model_silently_ignores_missing(tmp_path):
    copy_runtime_model("/nonexistent/model", str(tmp_path / "dst"))


def test_copy_runtime_model_uses_dirs_exist_ok(tmp_path):
    src = tmp_path / "src"
    src.mkdir()
    (src / "a.txt").write_text("A")
    dst = tmp_path / "dst"
    dst.mkdir()
    (dst / "b.txt").write_text("B")
    copy_runtime_model(str(src), str(dst))
    assert (dst / "a.txt").exists()
    assert (dst / "b.txt").exists()


# ---------------------------------------------------------------------------
# build_gz_resource_path
# ---------------------------------------------------------------------------

def test_build_gz_resource_path_no_existing(monkeypatch):
    monkeypatch.delenv("GZ_SIM_RESOURCE_PATH", raising=False)
    result = build_gz_resource_path("/models", "/ws")
    assert "/models" in result
    assert "/ws/models" in result
    assert "/ws/worlds" in result
    assert "/ws/install" in result


def test_build_gz_resource_path_appends_existing(monkeypatch):
    monkeypatch.setenv("GZ_SIM_RESOURCE_PATH", "/existing")
    result = build_gz_resource_path("/rt", "/ws")
    assert result.endswith(":/existing")


def test_build_gz_resource_path_no_double_colon_when_empty(monkeypatch):
    monkeypatch.delenv("GZ_SIM_RESOURCE_PATH", raising=False)
    result = build_gz_resource_path("/rt", "/ws")
    assert "::" not in result


# ---------------------------------------------------------------------------
# build_gz_plugin_path
# ---------------------------------------------------------------------------

def test_build_gz_plugin_path_base(monkeypatch):
    monkeypatch.delenv("GZ_SIM_SYSTEM_PLUGIN_PATH", raising=False)
    assert build_gz_plugin_path() == "/opt/ros/jazzy/lib"


def test_build_gz_plugin_path_appends_existing(monkeypatch):
    monkeypatch.setenv("GZ_SIM_SYSTEM_PLUGIN_PATH", "/extra")
    result = build_gz_plugin_path()
    assert "/opt/ros/jazzy/lib" in result
    assert "/extra" in result


# ---------------------------------------------------------------------------
# prepare_runtime_world_sdf
# ---------------------------------------------------------------------------

_WORLD_SDF = """\
<sdf><world name="test_world">
  <model name="camera_overhead"><link/></model>
  <model name="camera_north"><link/></model>
  <model name="ur5"><link/></model>
  <include><uri>model://ur5_rg2</uri></include>
</world></sdf>
"""


def test_prepare_runtime_world_sdf_replaces_uri(tmp_path):
    world = tmp_path / "world.sdf"
    world.write_text(_WORLD_SDF, encoding="utf-8")
    result = prepare_runtime_world_sdf(
        str(world), str(tmp_path), "/runtime/ur5_rg2",
        headless=False, keep_cameras=True,
    )
    assert result == str(tmp_path / "world_runtime.sdf")
    content = (tmp_path / "world_runtime.sdf").read_text()
    assert "file:///runtime/ur5_rg2" in content
    assert "model://ur5_rg2" not in content


def test_prepare_runtime_world_sdf_strips_cameras_headless(tmp_path):
    world = tmp_path / "world.sdf"
    world.write_text(_WORLD_SDF, encoding="utf-8")
    prepare_runtime_world_sdf(
        str(world), str(tmp_path), "/rt",
        headless=True, keep_cameras=False,
    )
    content = (tmp_path / "world_runtime.sdf").read_text()
    assert "camera_overhead" not in content
    assert "camera_north" not in content
    assert "ur5" in content  # non-camera model preserved


def test_prepare_runtime_world_sdf_keeps_cameras_if_requested(tmp_path):
    world = tmp_path / "world.sdf"
    world.write_text(_WORLD_SDF, encoding="utf-8")
    prepare_runtime_world_sdf(
        str(world), str(tmp_path), "/rt",
        headless=True, keep_cameras=True,
    )
    content = (tmp_path / "world_runtime.sdf").read_text()
    assert "camera_overhead" in content


def test_prepare_runtime_world_sdf_fallback_on_missing(tmp_path):
    result = prepare_runtime_world_sdf(
        "/nonexistent.sdf", str(tmp_path), "/rt",
        headless=False, keep_cameras=False,
    )
    assert result == "/nonexistent.sdf"


# ---------------------------------------------------------------------------
# PANEL_ENV_DEFAULTS
# ---------------------------------------------------------------------------

def test_panel_env_defaults_non_empty():
    assert len(PANEL_ENV_DEFAULTS) > 0


def test_panel_env_defaults_all_strings():
    for name, default in PANEL_ENV_DEFAULTS:
        assert isinstance(name, str) and name
        assert isinstance(default, str)


def test_panel_env_defaults_no_duplicates():
    names = [name for name, _ in PANEL_ENV_DEFAULTS]
    assert len(names) == len(set(names)), "Duplicate entries in PANEL_ENV_DEFAULTS"


def test_panel_env_defaults_known_keys_present():
    names = {name for name, _ in PANEL_ENV_DEFAULTS}
    required = {
        "PANEL_PICK_DEMO_GRASP_DOWN_SEGMENT_Z_STEP_M",
        "PANEL_PICK_DEMO_ATTACH_XY_TOL_M",
        "PANEL_PICK_DEMO_CLOSE_CONFIRM_TIMEOUT_SEC",
        "PANEL_DIRECT_DEBUG_ROOT",
    }
    assert required <= names
