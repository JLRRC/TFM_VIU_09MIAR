#!/bin/bash
# ============================================================================
# check_python_deps.sh
# ============================================================================
# Ruta y propósito: /home/laboratorio/TFM/agarre_inteligente/check_python_deps.sh
# Verifica dependencias Python del proyecto.
# ============================================================================

set -euo pipefail

if [[ ! -f requirements.txt ]]; then
  echo "[ERROR] requirements.txt no encontrado"
  exit 1
fi

if [[ -z "${VIRTUAL_ENV:-}" ]]; then
  echo "[WARN] VIRTUAL_ENV no activo. Se recomienda: source venv/bin/activate"
fi

python3 - << 'PY'
import importlib
import sys

packages = {
    "torch": "torch",
    "torchvision": "torchvision",
    "numpy": "numpy",
    "pandas": "pandas",
    "matplotlib": "matplotlib",
    "yaml": "pyyaml",
    "PIL": "pillow",
    "sklearn": "scikit-learn",
}

missing = []
for mod, name in packages.items():
    try:
        m = importlib.import_module(mod)
        ver = getattr(m, "__version__", "unknown")
        print(f"[OK] {name}: {ver}")
    except Exception:
        missing.append(name)
        print(f"[MISSING] {name}")

if missing:
    print("\nFaltan paquetes:", ", ".join(missing))
    sys.exit(1)
PY
