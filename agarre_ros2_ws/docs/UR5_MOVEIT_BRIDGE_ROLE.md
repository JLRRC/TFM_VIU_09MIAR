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
