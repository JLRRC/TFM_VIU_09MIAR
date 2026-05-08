#!/usr/bin/env python3
"""F4 audit-v4 (2026-05-08): inventory test for IDL contracts.

Verifica que cada srv/action declarado:
  1. Tiene formato canónico (request/response separator '---', tipo válido).
  2. Está listado en CMakeLists.txt rosidl_generate_interfaces.
  3. El paquete declara las dependencias mínimas (geometry_msgs).

NO valida la generación real del IDL (eso requiere ROS); valida el
contrato textual.
"""
from __future__ import annotations

import re
from pathlib import Path

import pytest

PKG_DIR = Path(__file__).resolve().parent.parent
SRV_DIR = PKG_DIR / "srv"
ACTION_DIR = PKG_DIR / "action"
CMAKE = PKG_DIR / "CMakeLists.txt"


# ---- Listado canónico (los que se exportan en CMakeLists.txt) ----

CANONICAL_SRVS = {
    "Attach.srv",
    "Close.srv",
    "ComputeApproachPose.srv",
    "Detach.srv",
    "Open.srv",
    "ResolveObjectPoseWorld.srv",
    "SelectObject.srv",
    "SetWidth.srv",
    "WorldToBase.srv",
}
CANONICAL_ACTIONS = {
    "PickPlace.action",
    "PlanToPose.action",
}


def _read(path: Path) -> str:
    return path.read_text(encoding="utf-8")


# ---- Estructura de paquete ----


def test_srv_dir_exists() -> None:
    assert SRV_DIR.is_dir(), f"srv dir missing: {SRV_DIR}"


def test_action_dir_exists() -> None:
    assert ACTION_DIR.is_dir(), f"action dir missing: {ACTION_DIR}"


def test_cmakelists_exists() -> None:
    assert CMAKE.is_file(), f"CMakeLists.txt missing"


def test_cmakelists_uses_rosidl_generate_interfaces() -> None:
    txt = _read(CMAKE)
    assert "rosidl_generate_interfaces" in txt, (
        "CMakeLists.txt doesn't call rosidl_generate_interfaces"
    )


def test_cmakelists_finds_geometry_msgs() -> None:
    txt = _read(CMAKE)
    assert "find_package(geometry_msgs" in txt, (
        "geometry_msgs not declared as build dep"
    )


# ---- Inventario srv ----


@pytest.mark.parametrize("srv", sorted(CANONICAL_SRVS))
def test_srv_file_present_and_listed_in_cmake(srv: str) -> None:
    path = SRV_DIR / srv
    assert path.is_file(), f"{srv} missing on disk"
    cmake_txt = _read(CMAKE)
    assert f'"srv/{srv}"' in cmake_txt, f"{srv} not listed in CMakeLists.txt"


@pytest.mark.parametrize("srv", sorted(CANONICAL_SRVS))
def test_srv_has_request_response_separator(srv: str) -> None:
    txt = _read(SRV_DIR / srv)
    assert "---" in txt, f"{srv} missing request/response '---' separator"


# ---- Inventario action ----


@pytest.mark.parametrize("action", sorted(CANONICAL_ACTIONS))
def test_action_file_present_and_listed_in_cmake(action: str) -> None:
    path = ACTION_DIR / action
    assert path.is_file(), f"{action} missing on disk"
    cmake_txt = _read(CMAKE)
    assert f'"action/{action}"' in cmake_txt, (
        f"{action} not listed in CMakeLists.txt"
    )


@pytest.mark.parametrize("action", sorted(CANONICAL_ACTIONS))
def test_action_has_two_separators(action: str) -> None:
    """Actions must have goal --- result --- feedback (two separators)."""
    txt = _read(ACTION_DIR / action)
    sep_count = len(re.findall(r"^---\s*$", txt, flags=re.MULTILINE))
    assert sep_count == 2, (
        f"{action} expected 2 '---' separators (goal/result/feedback), "
        f"found {sep_count}"
    )


def test_no_extra_srv_files() -> None:
    """Detect orphan srv files (file exists but not in canonical set)."""
    on_disk = {p.name for p in SRV_DIR.glob("*.srv")}
    extras = on_disk - CANONICAL_SRVS
    assert not extras, f"Orphan srv files: {extras}"


def test_no_extra_action_files() -> None:
    on_disk = {p.name for p in ACTION_DIR.glob("*.action")}
    extras = on_disk - CANONICAL_ACTIONS
    assert not extras, f"Orphan action files: {extras}"
