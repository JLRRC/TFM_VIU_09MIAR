# BUG — Controller feedback hang post-F1.18

**Estado**: ABIERTO — bug profundo de simulación (no de la lógica del orchestrator)
**Detectado**: 2026-05-08 14:17 (validación live de F1.18, HEAD `744006a`)
**Última auditoría**: `auditoria/audit_profesional_20260508_v5.md`
**Bugs relacionados**:
- [BUG_BRIDGE_PATH_TOLERANCE.md](./BUG_BRIDGE_PATH_TOLERANCE.md) (síntoma adyacente)
- [BUG_FJT_GOAL_TIME_TOLERANCE.md](./BUG_FJT_GOAL_TIME_TOLERANCE.md) (cerrado en F1.11)

## Síntoma

En cycles 2/3 del run E2E live `cycles 1/3` post-F1.18:

```
[PLAN_TO_POSE][MOVEIT_DIRECT] sending goal
target=(0.500,0.100,0.300) ee_frame=rg2_pinch_center group=manipulator
[move_group]: Starting trajectory execution ...
< 120s sin más logs >
[PLAN_TO_POSE][MOVEIT_DIRECT] first attempt hang
timeout=120.0s — cancelando goal y disparando retry
```

El `move_group` **acepta** el goal y arranca la ejecución, pero el feedback de
"trajectory completed" nunca llega a `plan_to_pose_server`. El goal acaba
cancelándose por `_MOVEIT_FIRST_ATTEMPT_TIMEOUT_SEC`.

## Lo que F1.18 SÍ resuelve

- Per-fase scaling (TRANSPORT 0.5, otras 0.25). Confirmado por test offline
  `test_classify_phase_*` (7 tests verde).
- Per-fase first_attempt_timeout (TRANSPORT 240s, otras 120s).
- Heurística pura testeable (`classify_phase_by_target_z`).

## Lo que F1.18 NO resuelve (controller recovery)

El feedback hang sigue ocurriendo en cycles 2/3 incluso con scaling=0.5 y
timeout=240s. El problema **no es** la duración de la trayectoria, sino la
ausencia de feedback del controller.

### Hipótesis (no validadas live)

| H | Hipótesis | Cómo verificar | Coste fix |
|---|-----------|----------------|-----------|
| H1 | Race en `controller_manager` post-attach (lock interno) | Inspeccionar logs de `controller_manager` en cycle 2 | Medio |
| H2 | `joint_trajectory_controller` deja de aceptar goals tras attach físico | `ros2 control list_controllers` post-cycle 1 | Medio |
| H3 | `gz_ros2_control` plugin cuelga en `update()` post-detach | `gz topic -e /clock` durante el hang | Alto |
| H4 | `move_group` cliente cachea handle stale del controller | restart de `move_group` entre cycles | Bajo |
| H5 | Zombies de `move_group` (3 instancias detectadas en F1.16) | `pgrep -c move_group` en CI | Bajo |

**H5 ya tiene fix parcial**: `cleanup_zombies.sh` ampliado (commit F1.16).

## Mitigaciones offline implementadas

1. **F1.18 `classify_phase_by_target_z`** — función pura con tests offline.
2. **first_attempt_timeout corto** (120-240s) detecta el hang sin
   bloquear al orchestrator hasta `_moveit_result_timeout` completo (400s).
3. **Retry con sleep 20s** post-cancel para dar tiempo al controller a
   recuperarse (F1.12).

## Plan de fix (próxima sesión LIVE)

1. Lanzar stack vivo + ejecutar 1 cycle E2E completo.
2. Inmediatamente despues del cycle 1 (antes de cycle 2): inspeccionar
   `ros2 control list_controllers --type` y `pgrep -c move_group`.
3. Si H5: confirmar que `cleanup_zombies.sh` se llama entre cycles del E2E
   driver (verificar `c3d4372 feat(F1.13): canonical e2e cycles driver`).
4. Si H4: añadir `move_group` restart entre cycles del driver.
5. Si H1/H2: añadir `/controller_manager/switch_controller` deactivate +
   activate de `joint_trajectory_controller` en el retry path de
   `plan_to_pose_server` cuando `reason ∈ {FIRST_ATTEMPT_TIMEOUT,
   CONTROL_FAILED}` y attempts > 1.
6. Si H3: bug de gz_ros2_control upstream — abrir issue.

## Tests guardrail (offline-friendly)

- ✅ `test_classify_phase_*` — 7 tests sobre la heurística (PASS HEAD post-F1.18)
- ✅ `test_build_goal_default_scaling_is_0_25` — default invariante (PASS)
- ✅ `test_build_goal_transport_scaling_0_5` — TRANSPORT scaling (PASS)
- 🔴 T31 contract test `path_tolerance MoveIt↔controller` — **PENDIENTE**
- 🔴 T33 guardrail "0 zombies move_group" — **PENDIENTE**
- 🔴 T35 3 cycles E2E consecutivos — **PENDIENTE LIVE**

## Referencias

- Commit F1.18: `744006a fix(F1.18): per-phase scaling/timeout — TRANSPORT detectado por Z<0.05`
- Commit F1.16: `cleanup_zombies.sh ampliado` (memoria 2026-05-08)
- Commit F1.13: `c3d4372 feat(F1.13): canonical e2e cycles driver con hard reset HOME`
- Auditoría v5: `auditoria/audit_profesional_20260508_v5.md` (a generar)
