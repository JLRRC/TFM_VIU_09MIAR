# T35 — Guía de activación: 3 ciclos E2E consecutivos

**Fecha**: 2026-05-08
**Estado**: smoke offline ✅ (18 tests) — activación live pendiente
**Test live**: [test_e2e_pick_cycles.py](../src/ur5_bringup/test/test_e2e_pick_cycles.py)
**Test smoke**: [test_e2e_pick_cycles_smoke.py](../src/ur5_bringup/test/test_e2e_pick_cycles_smoke.py)

## Objetivo académico

T35 es **el** test de validación final del TFM. Al pasar 3 ciclos consecutivos
limpios, el proyecto está listo para defensa con confianza máxima:

- Cycle 1: confirma que el primer pick post-arranque funciona (ya conseguido,
  HEAD `32119d0` 2026-05-08).
- Cycles 2/3: confirman que el sistema **se recupera** del primer cycle
  (controller_manager, MoveIt, gripper, attach_backend, world TF) y funciona
  consistentemente — esto es lo que falla actualmente por el bug residual
  documentado en [BUG_CONTROLLER_FEEDBACK_HANG.md](./BUG_CONTROLLER_FEEDBACK_HANG.md).

## Cobertura offline (smoke, 18 tests)

El smoke garantiza que el test live es estructuralmente correcto:

- `test_e2e_test_file_exists` / `test_e2e_module_loads_offline` — file + parse.
- `test_pass_pattern_matches_*` (3 markers PASS): legacy / ORCH legacy / ORCH lifecycle.
- `test_fail_pattern_matches_*` (9 markers FAIL): ABORT, FAIL_CLASS, ORCH/LC false,
  carry_coherence_failed, APPROACH_COARSE_NOT_READY, Error en pick.
- `test_skipif_disables_when_no_live_env` — guardrail del skip por defecto.
- `test_default_panel_env_*` — offscreen + auto-release activos.

Si añades un nuevo marker de PASS o FAIL al panel/orchestrator, **añade
también un caso a este smoke** o el regex se corromperá silenciosamente.

## Activación live (paso a paso)

### Pre-requisitos

- Estación con GPU disponible (Gazebo Sim Harmonic + RViz).
- Tag de seguridad pre-test: `git tag pre-T35-<fecha>`.
- Build limpio: `cd agarre_ros2_ws && colcon build`.
- Entorno: `source install/setup.bash` (después del build).

### Comando canónico

```bash
cd /home/laboratorio/TFM/agarre_ros2_ws
PICK_E2E_LIVE=1 \
PICK_VALIDATE_CYCLES=3 \
PICK_VALIDATE_WAIT_LOOPS=900 \
python3 -m pytest src/ur5_bringup/test/test_e2e_pick_cycles.py -q -s
```

Notas:
- `PICK_VALIDATE_CYCLES=3` — fija explícitamente 3 ciclos (default).
- `PICK_VALIDATE_WAIT_LOOPS=900` — 15 min por ciclo (post-F1.18 con
  TRANSPORT 240s + retry 240s + buffer).
- El test invoca `scripts/start_panel_v2.sh --bg` y `scripts/stop_panel_v2.sh`
  internamente.

### Variables de tuning relevantes

| Variable | Default | Cuándo cambiar |
|----------|---------|----------------|
| `PICK_E2E_LIVE` | `0` | Siempre `1` para activar |
| `PICK_VALIDATE_CYCLES` | `3` | `1` para smoke rápido, `5+` para stress |
| `PICK_VALIDATE_WAIT_LOOPS` | `600` | Subir a `900` por F1.18 (TRANSPORT lento) |
| `PICK_VALIDATE_REQUIRE_PASS` | `1` | `0` para debug (warning en lugar de FAIL) |
| `PANEL_FORCE_OFFSCREEN` | `1` | `0` para inspeccionar UI (requiere DISPLAY) |
| `PANEL_AUTO_RELEASE_DROP_OBJECTS` | `1` | Imprescindible para cycles consecutivos |

### Output esperado

PASS:
```
[PICK][DIRECT] SECUENCIA COMPLETADA EXITOSAMENTE route=basket   (cycle 1)
[PICK][DIRECT] SECUENCIA COMPLETADA EXITOSAMENTE route=basket   (cycle 2)
[PICK][DIRECT] SECUENCIA COMPLETADA EXITOSAMENTE route=basket   (cycle 3)
=== 1 passed in NNN.Ns ===
```

FAIL típico (bug actual):
```
cycle 1 OK
cycle 2: [ORCHESTRATOR_LC] result success=False reason=FIRST_ATTEMPT_TIMEOUT:120.0s
```

## Diagnóstico de fallos

### Si cycle 2 falla por `FIRST_ATTEMPT_TIMEOUT`

Es el bug residual de F1.18 documentado en
[BUG_CONTROLLER_FEEDBACK_HANG.md](./BUG_CONTROLLER_FEEDBACK_HANG.md).

Pasos de mitigación:
1. Ejecutar `pgrep -c move_group` durante la ejecución del cycle 2 (en
   otro terminal). Si > 1, hay zombies — F1.16 partial fix activo.
2. `ros2 control list_controllers` post-cycle 1 para ver si
   `joint_trajectory_controller` sigue activo.
3. Capturar logs de `gz_ros2_control` en `log/ros2_launch.log` durante
   el hang.
4. Si el patrón se confirma, escalar a las hipótesis H1-H5 de
   `BUG_CONTROLLER_FEEDBACK_HANG.md`.

### Si cycle 1 falla por `APPROACH_COARSE_NOT_READY`

Posible regresión del bug TF stale o gz_pose. Revisar:
- `world_tf_publisher` activo en `ros2 node list`
- `gz_pose_bridge` activo
- Memoria `project_world_tf_publisher_fix_20260427.md`

## Tags y commits relevantes

- `objetivo-cumplido-pinzas-agarran-objeto-20260507` — pick físico funciona.
- `744006a fix(F1.18): per-phase scaling/timeout` — TRANSPORT mitigado.
- `3942458 refactor(F1.18 cierre offline)` — heurística testeable.
- `0dbaf58 feat(F3-step40)` — seed_metrics + decision_helpers.
- `118c2a5 test(T31)` — path_tolerance contract.
- `<HEAD>` — T35 smoke offline (este doc + 18 tests guardrail).

## Criterio de éxito académico

El TFM se considera **defendible al máximo** cuando:

1. ✅ T35 smoke offline verde (18/18) — HOY.
2. 🔴 T35 live con `PICK_VALIDATE_CYCLES=3` y `success=True` × 3 — pendiente.
3. 🔴 T35 live con `PICK_VALIDATE_CYCLES=5` y `success=True` × 5 (stress) — pendiente.
4. ✅ T22 URDF↔SDF parity guardrail verde — HOY.
5. ✅ T31 path_tolerance contract verde — HOY.

Indicador único: **3 cycles consecutivos limpios**.
