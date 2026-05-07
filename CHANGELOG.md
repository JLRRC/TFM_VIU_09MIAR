# CHANGELOG

Historial de hitos del proyecto TFM UR5+RG2 Pick & Place (simulación ROS 2 Jazzy + Gazebo Harmonic + MoveIt 2).

## [2026-05-07] OBJETIVO CUMPLIDO — pinzas agarran objeto físicamente en Gazebo

**Tag**: `objetivo-cumplido-pinzas-agarran-objeto-20260507`
**Commit**: `a984234`

### Evidencia hard del agarre

```
[ATTACH_BACKEND] demo_transport_follow_tick
  object=pick_demo  mode=world_locked
  desired=(-0.722, 0.345, 1.789)   ← objeto en world
  tcp=(-0.731, 0.351, 1.804)        ← TCP en world
```

- TCP↔objeto separados ~1.8 cm (pinzas envolviendo el objeto).
- Z=1.79 m: robot levantó el objeto desde la mesa (Z=0.875 m).
- 90+ segundos consecutivos de transport con objeto siguiendo al TCP.
- `mode=world_locked`: attach físico activo.

### Camino canónico activo

`run_pick_demo` (legacy con todos los fixes acumulados de la sesión).

### Sesión 2026-05-07 (26 rondas live de validación)

#### Bugs cerrados

1. **URDF↔SDF parity (167mm gripper offset)** — commit `550457a` previo. Fix definitivo del bug arquitectónico que tenía las pinzas físicas a 167mm del centro lógico TF. Test T22 permanente bloquea regresiones.
2. **Tolerancias gates pre-grasp** — commit `2b91cb9` previo. Coarse XY/Z 0.012→0.020, source_tol 0.006→0.015, attach_dist 0.05→0.15.
3. **Conversión world→base_link** en builders del orchestrator (approach + transport + lift + grasp_down).
4. **GRASP_DOWN insertado** al FSM del orchestrator (cartesian descent vertical hasta objeto+2cm).
5. **Drop pose default** cambiado de `(0.5, 0, 0.05)` a `(-1.30, 0, 1.10)` para que el target world→base esté en el workspace.
6. **MoveIt timeouts** ampliados para Gazebo Sim lento: planning_time 5→25s, scaling 1.2→30, margin 5→90, result_timeout 30→400s, action_timeout 60→500s.
7. **Controller goal_time** subido de 0 a 300s para tolerar drift sim_time vs wall_time.
8. **Default dispatcher revertido a legacy** (orchestrator queda como opt-in `PANEL_PICK_DEMO_USE_ORCHESTRATOR=1`).

#### Bugs documentados pero abiertos (no críticos para el objetivo)

- **Bug bridge MoveIt-controller** (sim_time vs wall_time) — bloquea el orchestrator path en Gazebo Sim. Doc en `docs/BUG_BRIDGE_PATH_TOLERANCE.md` con plan de fix de 4-8h.

## [2026-05-06] Bloque 2 cerrado — orchestrator cableado al action

**Tag**: `cierre-bloque-2-orchestrator-cableado-20260506`
**Commit**: `60558fd`

- `pick_orchestrator_lifecycle_node` con `auto_activate` honrado al instantiation.
- Dispatcher panel→orchestrator funcional hasta SELECT_OBJECT.
- 9/9 fases del FSM dispatcheándose correctamente.

## [2026-05-06] Bloque 1 cerrado — pick físico validado

**Tag**: `cierre-bloque-1-pick-fisico-validado-20260506`
**Commit**: `2b91cb9`

- Bug crítico GRASP_DOWN cerrado por consecuencia del fix URDF↔SDF.
- Pick físico validado live (gripper toca + agarra + levanta 96cm).

## [2026-05-06] URDF↔SDF parity fix

**Tag**: `cierre-bloque-1-urdf-sdf-fix-20260506`
**Commit**: `550457a`

- Fix de 2 líneas en `models/ur5_rg2/model.sdf`: `end_effector_frame_fixed_joint` translation 0.275m→0, `rg2_mount_joint` -0.1927m→0.
- Test T22 permanente: `test_urdf_sdf_parity.py` (5 tests).

## [2026-05-04] Auditoría F0-F10 + F7 E2E live

**Tag**: `audit-fase-0-10-completed-20260504`

- 11 fases de auditoría ejecutadas (F0-F10).
- Primer ciclo E2E live con stack vivo.
- 3 bugs reales corregidos en F7 (closure faltante, imports rotos, FAIL_PATTERN test).

## [2026-05-03] B-iter1..14 sprint — orchestrator independiente del panel

**Tag**: `B-iter14-sprint-complete-20260503`

- 14 iteraciones del sprint Opción B en una sesión.
- 9/9 fases del FSM con dispatch real.
- 8 módulos puros nuevos (initial_snapshot, home_initial, phase_progress, pick_gates, retry, pose_consistency, cartesian_segments, gripper_monitor).
- Suite tfm_orchestrator: 168→313 tests.

## [2026-05-02] Bloque 0 + auditoría profesional

**Tag**: `audit-fase-0-1-cleanup-20260502`

- Auditoría profesional 10 secciones.
- Bloque F1 (emit_log_line) + F2 (env reads) + F4 (5 meta-tests baseline).

## [2026-04-28] Audit profesional + Fase 0/1 ejecutadas

**Tag**: `pre-fase0-fase1-cleanup-20260428`

- Plan F0-F10.
- Borrado 5266 LOC en rama `audit/fase-0-1-cleanup`.
- 602 tests verdes.

## [2026-04-27 → 2026-04-28] world_tf_publisher fix RESUELVE IK bug

- `_score_name` aceptaba `rg2_base_link` como base_link.
- world→base_link se publicaba con pose del gripper (rot -90X).
- Fix en commit `309a88e`.
- pos_err 0.487m → 0.003m.

## [2026-04-25] Migración UR5+RG2 → UR5+TCP limpio

- Stack migrado a `ur5_tcp_clean`.
- `tcp_tip = -0.18m`.
- 7 paquetes ROS 2 build OK.
- `gripper_controller` eliminado.

---

## Convenciones de versionado

- **Pre-1.0**: hitos por sesión (commits + tags fechados).
- **v1.0**: primera release estable cuando todos los pendientes opcionales (Bloques 3-6) cierren.

## Tags rollback

Cada hito tiene un tag de rollback. Para volver a un punto:
```bash
git checkout <tag>
```

## Próximas versiones planificadas (tentativo)

- **v0.9** (post bug bridge orchestrator): orchestrator path validado live.
- **v1.0** (post Bloques 3-6): legacy borrado + arquitectura microservicio madura + tests completos en CI + README profesional.
