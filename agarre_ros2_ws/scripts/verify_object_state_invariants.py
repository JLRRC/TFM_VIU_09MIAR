#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/scripts/verify_object_state_invariants.py
# Contenido: Script operativo o de diagnostico del workspace ROS 2.
# Uso breve: Se ejecuta manualmente para arrancar, medir o validar el stack.
"""Quick verification of panel object-state invariants (no ROS runtime needed)."""
from __future__ import annotations

import os
import sys


def _die(msg: str, code: int = 2) -> None:
    print(msg, file=sys.stderr)
    raise SystemExit(code)


def _ensure_import() -> None:
    try:
        import ur5_qt_panel.panel_objects as _  # noqa: F401
    except Exception:
        _die(
            "ERROR: cannot import ur5_qt_panel.panel_objects.\n"
            "Run with:\n"
            "  PYTHONPATH=/home/laboratorio/TFM/agarre_ros2_ws/src/ur5_qt_panel "
            "python agarre_ros2_ws/scripts/verify_object_state_invariants.py"
        )


def main() -> int:
    _ensure_import()
    import ur5_qt_panel.panel_objects as panel_objects

    ok = True
    print("== Verificando invariantes ObjectState ==")

    # TEST read-only: no pose updates must be applied.
    updated = panel_objects.bulk_update_object_positions(
        {"pick_demo": (0.0, 0.0, 0.0)},
        source="test_ro",
        read_only=True,
    )
    if updated != 0:
        ok = False
        print("FAIL: read_only allowed pose update")
    else:
        print("OK: read_only blocks pose update")

    # Enforce pick_demo sequence: ON_TABLE -> GRASPED -> CARRIED -> RELEASED.
    if not panel_objects.mark_object_grasped("pick_demo", reason="demo"):
        ok = False
        print("FAIL: GRASPED rejected for pick_demo")
    if not panel_objects.mark_object_attached("pick_demo", reason="demo"):
        ok = False
        print("FAIL: CARRIED rejected for pick_demo")
    if not panel_objects.mark_object_released("pick_demo", reason="demo"):
        ok = False
        print("FAIL: RELEASED rejected for pick_demo")
    else:
        print("OK: pick_demo sequence accepted")

    print("== Resultado ==")
    if ok:
        print("PASS: invariantes OK")
        return 0
    print("FAIL: invariantes violadas")
    return 1


if __name__ == "__main__":
    sys.exit(main())
