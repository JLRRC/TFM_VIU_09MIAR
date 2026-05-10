# Cierre tecnico: pick fisico UR5 + RG2 y regresiones

Fecha: 2026-05-10

## Decision

Se considera cerrada la reparacion operacional del pick fisico de `pick_demo` en Gazebo: la evidencia valida no es el panel, sino el movimiento fisico del objeto durante agarre, lift y transporte.

Los directorios completos bajo `reports/geometry_physical_pick_repair_*`, `reports/geometry_physical_pick_repair_cont_*` y `reports/stabilize_geometry_moveit_tests_*` quedan como artefactos locales generados y no se versionan por defecto. La trazabilidad versionable queda en este documento, en los cambios de codigo y en los scripts de regresion.

## Evidencia principal local

Directorio local de evidencia: `reports/geometry_physical_pick_repair_cont_20260510_132730/`.

Resumen validado:

| Check | Valor |
|---|---:|
| Resultado action | `SUCCESS` |
| Razon | `ok` |
| Ciclos completados | `1` |
| Distancia TCP-objeto en attach | `0.0205 m` |
| Mejor lift del objeto | `0.2880 m` |
| Movimiento maximo del objeto | `0.9328 m` |
| Distancia TCP durante follow | `0.0151 m` |
| Ticks de seguimiento | `38` |

Fases observadas: `APPROACH`, `GRASP_DOWN`, `GRASP`, `LIFT`, `TRANSPORT`, `RELEASE`, `DONE`.

## Cambios versionables asociados

- `scripts/run_single_pick_pickdemo.py`: actualiza el runner al esquema vigente de `PickPlace.action`, envia `object_pose_world_hint` y evita drop por defecto en `(0,0,0)`.
- `src/ur5_tools/ur5_tools/plan_to_pose_server.py`: parametriza timeout de envio a MoveIt y protege FJT directo con umbral de salto articular IK.
- `src/ur5_bringup/launch/*.py` y `src/ur5_bringup/config/runtime_defaults.yaml`: estabilizan arranque headless sin camaras/render pesado y activan MoveIt por defecto.
- `scripts/test_geometry_regression.sh`, `scripts/test_moveit_regression.sh`, `scripts/test_pick_physics_regression.sh`, `scripts/run_tfm_regression_suite.sh`: wrappers de regresion.

## Validacion de regresion

Comando ejecutado:

```bash
bash scripts/run_tfm_regression_suite.sh
```

Resultado:

```text
[GEOMETRY_REGRESSION] 72 passed
[MOVEIT_REGRESSION] 53 passed
[PICK_PHYSICS_REGRESSION][SKIP] Set TFM_PHYSICAL_PICK_LIVE=1 to run Gazebo physical pick.
[TFM_REGRESSION] OK
```

La regresion fisica live queda opt-in:

```bash
TFM_PHYSICAL_PICK_LIVE=1 bash scripts/run_tfm_regression_suite.sh
```

## Riesgo pendiente

La ruta MoveIt pura de `APPROACH` sigue expuesta a una colision conservadora `mesa_pro - rg2_finger_link1`. El flujo fisico validado funciona mediante la ruta operacional con FJT directo y guardia IK; no debe describirse como una solucion completa de planificacion MoveIt pura.