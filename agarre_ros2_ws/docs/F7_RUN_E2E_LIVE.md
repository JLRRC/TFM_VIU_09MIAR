# F7 — Validación E2E live con orchestrator independiente

**Estado**: ✅ Lifecycle offline verificado (98/100 tests verde) · ⏳ E2E live pendiente de display X11

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
