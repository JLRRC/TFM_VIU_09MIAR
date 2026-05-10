#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/scripts/prepare_runtime.py
# Contenido: F8 audit (2026-05-10) — preparación de runtime artefacts pre-launch.
"""F8 audit (2026-05-10): prepara los artifacts runtime del stack UR5+RG2.

Antes (pre-F8): un OpaqueFunction ``_prepare_runtime`` en
``ur5_stack.launch.py`` ejecutaba en launch-time (3-5 min de overhead):
  * patch del bridge YAML con world_name dinámico
  * copia y patching del modelo SDF en ``log/gz_models/``
  * generación de runtime_world.sdf con cameras stripped si headless

Ahora (F8): este script puede ejecutarse antes del ``ros2 launch`` para
generar los artifacts ofuscando ese trabajo del path crítico de
arranque. El launch sigue funcionando sin pre-ejecutar el script (el
OpaqueFunction lo hace bajo demanda como fallback) pero el modo
"prepare antes" reduce el tiempo de startup percibido.

Uso (manual, antes del launch)::

    python3 scripts/prepare_runtime.py
    ros2 launch ur5_bringup ur5_stack.launch.py

Uso (CI / scripts)::

    python3 scripts/prepare_runtime.py --headless --world-file ...
    PRE_PREPARED=1 ros2 launch ur5_bringup ur5_stack.launch.py

El test ``test_prepare_runtime.py`` valida la idempotencia.
"""
from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path
from typing import Optional


def _resolve_paths(ws_dir: Path) -> dict:
    """Resuelve los paths del workspace (ur5_gazebo + log)."""
    src = ws_dir / "src"
    return {
        "ws_dir": ws_dir,
        "log_dir": ws_dir / "log",
        "ur5_gazebo_src": src / "ur5_gazebo",
        "worlds_src": src / "ur5_gazebo" / "worlds",
        "models_src": src / "ur5_gazebo" / "models",
        "config_src": src / "ur5_gazebo" / "config",
    }


def _ensure_log_dir(log_dir: Path) -> None:
    log_dir.mkdir(parents=True, exist_ok=True)
    (log_dir / "gz_models").mkdir(exist_ok=True)


def _resolve_world_file(paths: dict, world_arg: Optional[str]) -> Path:
    """Devuelve el path absoluto al world SDF a usar."""
    if world_arg:
        p = Path(world_arg).expanduser().resolve()
        if not p.is_file():
            raise FileNotFoundError(f"world file not found: {p}")
        return p
    default = paths["worlds_src"] / "ur5_mesa_objetos.sdf"
    if not default.is_file():
        raise FileNotFoundError(f"default world missing: {default}")
    return default


def main(argv: Optional[list] = None) -> int:
    parser = argparse.ArgumentParser(
        prog="prepare_runtime",
        description="F8: prepare runtime artifacts (bridge YAML, runtime SDF, gz_models).",
    )
    parser.add_argument(
        "--ws-dir",
        default=os.environ.get("WS_DIR", os.path.expanduser("~/TFM/agarre_ros2_ws")),
        help="Workspace root (default: $WS_DIR or ~/TFM/agarre_ros2_ws)",
    )
    parser.add_argument(
        "--world-file",
        default=None,
        help="World SDF a usar (default: ur5_gazebo/worlds/ur5_mesa_objetos.sdf)",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Strip cameras del world SDF (headless mode)",
    )
    parser.add_argument(
        "--keep-cameras",
        action="store_true",
        help="Mantener cámaras incluso en headless (para diagnostic)",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Solo imprime lo que haría, sin escribir",
    )
    args = parser.parse_args(argv)

    ws_dir = Path(args.ws_dir).expanduser().resolve()
    if not ws_dir.is_dir():
        print(f"[prepare_runtime] ERROR: ws_dir no existe: {ws_dir}", file=sys.stderr)
        return 2

    paths = _resolve_paths(ws_dir)

    if not paths["ur5_gazebo_src"].is_dir():
        print(
            f"[prepare_runtime] ERROR: paquete ur5_gazebo no encontrado en "
            f"{paths['ur5_gazebo_src']}; ¿hiciste F5? F8 lo asume.",
            file=sys.stderr,
        )
        return 3

    _ensure_log_dir(paths["log_dir"])

    try:
        world_file = _resolve_world_file(paths, args.world_file)
    except FileNotFoundError as exc:
        print(f"[prepare_runtime] ERROR: {exc}", file=sys.stderr)
        return 4

    # Importamos los helpers ya existentes de launch_helpers para
    # mantener una sola fuente de verdad de la lógica.
    sys.path.insert(0, str(paths["ws_dir"] / "src" / "ur5_bringup" / "launch"))
    try:
        from launch_helpers import (
            patch_bridge_yaml,
            prepare_runtime_world_sdf,
            resolve_world_name,
        )
    except Exception as exc:  # pragma: no cover
        print(f"[prepare_runtime] ERROR import launch_helpers: {exc}", file=sys.stderr)
        return 5

    log_dir = str(paths["log_dir"])
    bridge_yaml = str(paths["config_src"] / "ros_gz_bridge.yaml")
    runtime_yaml = os.path.join(log_dir, "bridge_runtime.yaml")
    runtime_models_root = os.path.join(log_dir, "gz_models")
    runtime_ur5_model = os.path.join(runtime_models_root, "ur5_rg2")

    world_name = resolve_world_name(str(world_file))

    print(f"[prepare_runtime] ws_dir={ws_dir}")
    print(f"[prepare_runtime] world_file={world_file} (name={world_name})")
    print(f"[prepare_runtime] bridge_yaml: {bridge_yaml} → {runtime_yaml}")
    print(f"[prepare_runtime] runtime_models: {runtime_ur5_model}")

    if args.dry_run:
        print("[prepare_runtime] dry-run, no writes")
        return 0

    # 1) bridge YAML con world_name patched
    runtime_yaml_path = patch_bridge_yaml(bridge_yaml, runtime_yaml, world_name)
    print(f"[prepare_runtime] wrote runtime bridge YAML: {runtime_yaml_path}")

    # 2) runtime model copy + patch
    try:
        from launch_helpers import copy_runtime_model
    except Exception:
        copy_runtime_model = None
    if copy_runtime_model:
        copy_runtime_model(
            str(paths["models_src"] / "ur5_rg2"),
            runtime_ur5_model,
        )
        print(f"[prepare_runtime] copied runtime model")

    # 3) runtime world SDF (cameras stripped si headless)
    runtime_world_path = prepare_runtime_world_sdf(
        str(world_file),
        log_dir,
        runtime_ur5_model,
        headless=args.headless and not args.keep_cameras,
        keep_cameras=args.keep_cameras,
    )
    print(f"[prepare_runtime] wrote runtime world SDF: {runtime_world_path}")

    print("[prepare_runtime] OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
