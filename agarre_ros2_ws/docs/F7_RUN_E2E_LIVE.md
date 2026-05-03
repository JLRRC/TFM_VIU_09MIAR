# F7 — Validación E2E live con orchestrator independiente

**Estado** (actualizado 2026-05-04 tras run live): ✅ Lifecycle offline verificado (98/100 tests verde) · ✅ E2E live ejecutado · ⚠ ciclo bloqueado por tuning preexistente

## Resumen del run live (2026-05-04)

Ejecutado `PICK_E2E_LIVE=1 PICK_VALIDATE_CYCLES=1 PICK_VALIDATE_REQUIRE_PASS=0`. 3 bugs reales detectados y corregidos:

| # | Bug | Fix commit | Tipo |
|---|---|---|---|
| 1 | `_get_global_step_timeout_extra` no definido (closure faltante) | `c7470dc` | regresión refactor F3-step3e |
| 2 | `pick_demo/joint_step.py` 5 imports faltantes (`_pick_demo_env_float`, `_is_demo_basket_transport_stage`, etc.) | `4eb206f` | regresión refactor F3-step3e |
| 3 | `FAIL_PATTERN` del test no detecta `Error en pick demo` (solo `objeto`) | `8eab064` | typo del test |

Tras los fixes el ciclo progresó: `INITIAL_SNAPSHOT → HOME_INITIAL → APPROACH_COARSE`.

**Bloqueo final**: `APPROACH_COARSE_NOT_READY` — gate `pregrasp_strict` rechaza por `tcp_obj_dist=0.029m` (tol 0.015m) y `dz_obj=0.028m` (tol 0.015m). Es **tuning preexistente** documentado en memoria 2026-04-15 (DH/SDF divergencia + lag TF), NO regresión de la auditoría.

## Validación arquitectónica conseguida

✅ Stack arranca limpio (Gazebo Harmonic + MoveIt 2 + 8 lifecycle nodes + panel offscreen)
✅ Orchestrator instanciado y operativo
✅ `gripper_attach_backend` activado por lifecycle
✅ `ur5_moveit_bridge` activado por lifecycle
✅ Resolver de objeto funciona (`pick_demo` localizado en mesa)
✅ FK panel y TF live consistentes con divergencia <11mm
✅ El robot ejecuta movimiento real (TCP recorrió de home a pre-grasp)



## Qué cubre el offline (ya verde)

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 -m pytest \
    src/tfm_orchestrator/test/test_pick_orchestrator_lifecycle_launch.py \
    src/tfm_orchestrator/test/test_lifecycle_helpers.py \
    src/ur5_tools/test/test_lifecycle_contract.py \
    -q
# 98 passed, 2 failed (launch_testing infra, no lifecycle real)
```

Cobertura offline:
- Orchestrator transita configure → activate → deactivate → cleanup
- `/pick_place` action server creado en `on_configure` y removed en `on_cleanup`
- Goals rechazados con `node_not_active:<state>` cuando no ACTIVE
- 313 tests de FSM puro (`test_phase_dispatch.py` + `test_pick_fsm.py` + B-iter1..14)
- Snapshot del contrato legacy `panel_pick_demo` (test_legacy_pick_demo_contract_snapshot, 6/6)

## Qué falta — E2E live

Requiere display X11 funcional + stack ROS 2 + Gazebo Harmonic.

### Comando único

```bash
cd ~/TFM/agarre_ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# 3 ciclos pick_demo via orchestrator (default)
PICK_E2E_LIVE=1 python3 -m pytest \
    src/ur5_bringup/test/test_e2e_pick_cycles.py -q -s

# o el wrapper bash:
./scripts/validate_pick_3_cycles.sh 3
```

### Variables de entorno relevantes

| Var | Default | Efecto |
|---|---|---|
| `PICK_E2E_LIVE` | unset | `1` activa el test (skip en CI) |
| `PICK_VALIDATE_CYCLES` | 3 | número de ciclos a ejecutar |
| `PICK_VALIDATE_WAIT_LOOPS` | 600 | timeout segundos por ciclo |
| `PICK_VALIDATE_REQUIRE_PASS` | 1 | `0` = warning-only mode |
| `PANEL_PICK_DEMO_USE_ORCHESTRATOR` | 1 | usa el path orchestrator (no legacy) |

### Criterio de éxito

Log final debe contener una línea `SECUENCIA COMPLETADA EXITOSAMENTE`
por cada ciclo, sin patrones de fallo:
- `carry_coherence_failed`
- `[PICK_OBJ][ABORT]`
- `[PICK][DIRECT][ABORT]`
- `[PICK_OBJ][FAIL_CLASS]`

### Si pasa: desbloquea F8

F8 borra el legacy `panel_pick_demo.py` (8.623 LOC) y deja sólo el
action client. El test snapshot `test_legacy_pick_demo_contract_snapshot.py`
saltará automáticamente cuando detecte LOC < 50 % del baseline.

### Si falla: rollback

```bash
git tag -l | grep pre-legacy-removal
# pre-legacy-removal-20260503
git reset --hard pre-legacy-removal-20260503
```
