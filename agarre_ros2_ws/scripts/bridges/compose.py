#!/usr/bin/env python3
"""F9 audit (2026-05-10): compose ros_gz_bridge.yaml from per-domain fragments.

Lee todos los ``scripts/bridges/[0-9]*.yaml`` por orden alfabético y
escribe la concatenación canónica en ``scripts/ros_gz_bridge.yaml``.

Sustituye los antiguos comentarios por uno generado automáticamente.
El runtime sigue consumiendo ``ros_gz_bridge.yaml`` vía
``patch_bridge_yaml`` en ``launch_helpers.py``: el split es interno.

Uso::

    python3 scripts/bridges/compose.py        # actualiza ros_gz_bridge.yaml
    python3 scripts/bridges/compose.py --dry  # imprime sin escribir

Test ``test_bridge_yaml_compose_parity`` en ``ur5_bringup/test`` valida
que la composición coincide con el archivo en disco.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import List

import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent
TARGET = ROOT / "scripts" / "ros_gz_bridge.yaml"

HEADER = """\
# Ruta/archivo: agarre_ros2_ws/scripts/ros_gz_bridge.yaml
# AUTO-GENERADO por scripts/bridges/compose.py — NO EDITAR a mano.
# Editar los fragmentos en scripts/bridges/*.yaml y volver a ejecutar:
#
#     python3 scripts/bridges/compose.py
#
# patch_bridge_yaml() en launch_helpers.py copia este archivo al
# log/ de runtime sustituyendo el world_name dinámicamente.
"""


def _read_fragments() -> List[dict]:
    fragments: List[dict] = []
    for path in sorted(HERE.glob("[0-9]*.yaml")):
        loaded = yaml.safe_load(path.read_text(encoding="utf-8")) or []
        if not isinstance(loaded, list):
            raise ValueError(f"Fragment {path} no es una lista YAML.")
        fragments.extend(loaded)
    return fragments


def compose() -> str:
    entries = _read_fragments()
    body = yaml.safe_dump(entries, sort_keys=False, allow_unicode=True)
    return HEADER + body


def main(argv: List[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dry", action="store_true", help="No escribir, mostrar.")
    args = parser.parse_args(argv)
    out = compose()
    if args.dry:
        sys.stdout.write(out)
        return 0
    TARGET.write_text(out, encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
