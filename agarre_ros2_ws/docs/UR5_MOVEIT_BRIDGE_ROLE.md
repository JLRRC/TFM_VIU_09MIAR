# ur5_moveit_bridge.py — rol y futuro

**Última actualización**: 2026-05-08 (audit-v4 close)
**Tamaño actual**: 1.847 LOC, LifecycleNode

---

## Decisión post audit-v4: **conservar como fallback defensivo**

`ur5_moveit_bridge.py` queda en producción como **fallback defensivo no-canonical**. NO se adelgaza ni se elimina en v1.0.

### Por qué se conservó

1. **Path canónico actual (post-F1.24)**: el orchestrator usa **FJT directo** vía `plan_to_pose_server` con bypass de MoveIt para distancias cortas. Esto evita el bug `simple_controller_manager` no-feedback.
2. **Fallback explícito**: cuando `compute_ik` falla en `plan_to_pose_moveit_direct.py`, el código cae al path MoveIt completo, que pasa por `ur5_moveit_bridge`. Sin él, una IK fallida aborta el ciclo.
3. **Robot real futuro (F10)**: el plan F10 (UR5 físico) requiere control de trayectoria mediado por MoveIt servo, donde este bridge será el coordinador. Eliminarlo ahora obligaría a reescribirlo en F10.
4. **F13 LifecycleNode contract**: `ur5_moveit_bridge` ya pasa el contract test de transitions. Es un nodo "Observable" estable.

### Por qué NO se adelgaza ahora

- Mantenerlo separado del path canónico significa que **si T35 verde × 3 ciclos sigue verde** (path FJT directo), el bridge no afecta — no es código en uso productivo del cycle estándar.
- Adelgazarlo a "stub ≤ 300 LOC" eliminaría capacidades (MoveItPy planner, planning_scene tracking) que son útiles para debug y para el F10 robot real.
- Riesgo > beneficio en v1.0.

### Compromiso del codebase

- ✅ `ur5_moveit_bridge` **NO** está en el path canónico del cycle.
- ✅ Sólo se invoca como **fallback defensivo** cuando el FJT directo falla.
- ✅ Entry point sigue activo en `setup.py` para escenarios no-canónicos (manual MoveIt, debug, F10 robot real).
- ✅ Tests T31 (path_tolerance contract) y T-bridge regression cubren su superficie pública.

### Trabajo F10 (deferred a v1.1+)

Cuando se aborde F10 (robot real):
1. Auditar qué métodos del bridge son necesarios para HW (vs sim).
2. Extraer logic puro (joint settling math, retry strategy) a `moveit_bridge_logic.py`.
3. Mantener LifecycleNode delgado wrapper.
4. Tests dedicados para el camino HW.

### Diagrama actual

```
                     ┌────────────────────┐
                     │ plan_to_pose_server│
                     │ (canónico)         │
                     └─────┬──────────┬───┘
                           │          │
                  IK OK    │          │ IK FAIL
                           ▼          ▼
                    ┌──────────┐  ┌────────────────────┐
                    │ FJT      │  │ ur5_moveit_bridge  │
                    │ directo  │  │ (FALLBACK)         │
                    └────┬─────┘  └─────┬──────────────┘
                         │              │
                         └──────┬───────┘
                                ▼
                        /joint_trajectory_controller
```

### Métricas (HEAD audit-v4-release-logic-20260508)

- LOC: 1.847 (sin cambio en audit-v4).
- Coverage tests offline: T31 + T-bridge regression + lifecycle contract.
- Path uso: **fallback no-default** (FJT directo es default).
- Decisión: **mantener** hasta F10 (cuando se reescriba para HW real).

---

## Audit-v4.1 (2026-05-08) — hallazgos adicionales tras inspección de uso real

Esta sección **amplía** la decisión del audit-v4 con dos hechos nuevos detectados al investigar la FASE G del [AUDIT_20260508_v4_1.md](AUDIT_20260508_v4_1.md):

### G.1 — El bridge tiene un rol operacional residual no-fallback

`ur5_moveit_bridge` **se lanza por default** vía `stack_factories.py:255` (`DeclareLaunchArgument("launch_moveit_bridge", default_value="true")`) **no** como fallback opt-in. El comentario en [`runtime_nodes_factory.py:348-354`](../src/ur5_bringup/launch/runtime_nodes_factory.py#L348-L354) lo justifica explícitamente:

> *"Launch ur5_moveit_bridge as a standalone node so that ``/desired_grasp`` always has a real subscriber in the normal boot. Previously Subscription count was 0 because the bridge was only started on-demand via the panel button; this makes it part of the managed launch."*

Es decir: el bridge **ES** subscriber default de `/desired_grasp`. Sin él, el panel publica grasps a una topic con 0 subs y el flujo MoveIt manual del botón "Agarre Objeto (MoveIt)" no funciona.

### G.2 — La eliminación depende de cerrar `panel_pick_object.py`

El publisher principal de `/desired_grasp` es [`panel_pick_object.py`](../src/ur5_qt_panel/ur5_qt_panel/panel_pick_object.py) (3.814 LOC, último monolito real del audit-v4.1). Mientras ese path siga vivo, el bridge **debe seguir subscribed** al topic. La cadena de dependencias para una eventual eliminación del bridge es:

1. **F3-iter** completa el split de `panel_pick_object.py` → cliente Action `MoveItPick` (o reusar `PickPlace` con `mode=moveit_manual`).
2. La acción nueva es servida por `pick_orchestrator_lifecycle_node` (no por bridge subscriber).
3. `/desired_grasp` queda sin publisher activo → el bridge se puede borrar / migrar a `package_data`.

**Hasta entonces, mantener el bridge es OBLIGATORIO** (no opcional como sugería el audit-v4). El comentario "fallback defensivo" del audit-v4 es **incompleto**: el bridge tiene rol operacional activo en el flujo MoveIt manual del panel.

### G.3 — Decisión actualizada (v4.1)

| Aspecto | v4 (decisión) | v4.1 (refinamiento) |
|---|---|---|
| Estado | Conservar como fallback | Conservar como **subscriber operacional** + fallback |
| Adelgazar a ≤300 LOC | Postergado a F10 | **Bloqueado** hasta F3-iter cierre `panel_pick_object` |
| Borrar | F10 (robot real) | F3-iter completa + migración panel→Action client |
| Eliminar entry point | F10 | Cuando `/desired_grasp` quede sin publisher activo |
| Tests gating | Mantener todos | + LOC baseline 1.849 (FASE B audit-v4.1, ya activo) |

**Conclusión v4.1**: la decisión binaria del audit-v4 (slim ≤300 vs documentar fallback) es **falsa dicotomía**. La decisión correcta es **diferir** hasta que F3-iter cierre el publisher único; entonces se evalúa eliminación completa, no slim.

### G.4 — Métricas actualizadas (HEAD audit-v4.1 4df1389)

- LOC: 1.849 (+2 vs v4 — sólo nuevo import de `FjtLifecycleMixin` en cadena MRO).
- LOC baseline gating: 1.849 (`test_loc_baseline_ur5_tools.py`, FASE B audit-v4.1).
- Status FJT path bypass: cerrado (FASE A audit-v4.1, `executor.py` 1.546 LOC bajo el baseline v4).
- Importadores en código no-test: 27 (panel/pick_object/launch).
