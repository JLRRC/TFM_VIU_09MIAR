# Inventario de `os.environ` en producción (audit-v4.1, 2026-05-08)

**HEAD**: `e9cdceb` (post FASE A+B audit-v4.1)
**Total reads** (excluyendo `test/`, `cli_*`, `build/`, `install/`): **97**
**Documento parent**: [AUDIT_20260508_v4_1.md](AUDIT_20260508_v4_1.md) — FASE D

## Distribución por archivo

| Archivo | Reads | Categoría | Acción FASE D |
|---|:---:|---|---|
| `src/ur5_qt_panel/ur5_qt_panel/panel_pick_object.py` | 13 | DEUDA — monolito | Migrar tras split iter2 |
| `src/ur5_qt_panel/ur5_qt_panel/panel_env.py` | 12 | LEGÍTIMO — módulo central | excluir del gate |
| `src/ur5_bringup/launch/launch_helpers.py` | 8 | LEGÍTIMO — launch params (4 reales + 4 docstring/comentario) | excluir del gate |
| `src/ur5_qt_panel/ur5_qt_panel/panel_pick_demo_params.py` | 5 | LEGÍTIMO — `*_params.py` | excluir del gate |
| `src/ur5_qt_panel/ur5_qt_panel/panel_gz_startup.py` | 5 | DEUDA | migrar a `panel_env` |
| `src/ur5_tools/ur5_tools/release_objects_service.py` | 3 | DEUDA | migrar a `bridge_env_*` |
| `src/ur5_qt_panel/ur5_qt_panel/panel_settings.py` | 3 | DEUDA | migrar a `panel_env` |
| `src/ur5_qt_panel/ur5_qt_panel/panel_launchers.py` | 3 | DEUDA | migrar a `panel_env` |
| `src/ur5_qt_panel/ur5_qt_panel/directo_geometry.py` | 3 | DEUDA | migrar a `panel_env` |
| `src/ur5_moveit_config/launch/ur5_moveit_bringup.launch.py` | 3 | LEGÍTIMO — launch | excluir del gate |
| `src/ur5_tools/ur5_tools/moveit_bridge/moveit_commander_planner.py` | 2 | DEUDA | migrar a `bridge_env_*` |
| `src/ur5_tools/ur5_tools/gripper_geometry.py` | 2 | DEUDA | migrar a panel_env equivalente |
| `src/ur5_tools/ur5_tools/attach_set_pose.py` | 2 | DEUDA | migrar |
| `src/ur5_qt_panel/ur5_qt_panel/pick_demo_dispatcher.py` | 2 | DEUDA | migrar a `panel_env` |
| `src/ur5_qt_panel/ur5_qt_panel/panel_v2_helpers.py` | 2 | DEUDA | migrar a `panel_env` |
| `src/ur5_qt_panel/ur5_qt_panel/panel_utils.py` | 2 | DEUDA | migrar a `panel_env` |
| `src/ur5_qt_panel/ur5_qt_panel/panel_ros.py` | 2 | DEUDA | migrar a `panel_env` |
| `src/ur5_qt_panel/ur5_qt_panel/panel_config.py` | 2 | DEUDA | migrar a `panel_env` |
| `src/ur5_bringup/launch/stack_factories.py` | 2 | LEGÍTIMO — launch | excluir del gate |
| `src/tfm_grasping/tfm_grasping/model.py` | 2 | DEUDA | migrar a tfm_grasping/env |
| ...resto | 19 | mixto | revisar caso a caso |

## Categoría legítima vs deuda

**LEGÍTIMO** (patrón válido, no mover):
- `panel_env.py` y `*_params.py`: existen para encapsular env reads.
- `launch/*.py`: configuración de arranque vía environment variables es estándar en ROS 2 launch.
- `bridge_env_*` en `moveit_bridge_utils.py` (similar a `panel_env`).
- `_env_optional_str` / `_env_float` en `tfm_orchestrator/preflight.py`.

**DEUDA** (debe migrar):
- Cualquier `os.environ.get` fuera de los anteriores patrones.

## Plan de migración (FASE D.2, sesión dedicada)

| Tanda | Archivos | Reads | Esfuerzo | Riesgo |
|---|---|:---:|:---:|:---:|
| 1 | panel_settings, panel_launchers, panel_config, panel_ros, panel_utils, panel_v2_helpers, pick_demo_dispatcher | 16 | 3-4 h | bajo |
| 2 | panel_gz_startup, panel_pick_object, directo_geometry | 21 | 4-6 h | medio |
| 3 | ur5_tools/* (release_objects, moveit_commander_planner, gripper_geometry, attach_set_pose) | 9 | 2-3 h | bajo |
| 4 | tfm_grasping/model.py + 19 reads dispersas | 21 | 2-3 h | bajo |

**Objetivo final**: ~67 deuda real → 0, con gate `test_no_environ_outside_panel_env` blindando.

## Gate test (FASE D.3, instalado en este sprint)

`src/ur5_qt_panel/test/test_env_reads_allowlist.py` o equivalente: snapshot del set actual de archivos con env reads en producción. Test falla si:
1. Aparece un archivo nuevo con `os.environ.*` que no esté en la allowlist.
2. La allowlist tiene archivos que ya no contienen env reads (señal de que se pueden eliminar de la lista).

Esto blinda la deuda actual (no permite que crezca) sin requerir migración inmediata.
