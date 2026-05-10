# ur5_tools — Arquitectura

> F11 (auditoría 2026-05-10): organización por dominios. Los archivos
> físicos están todos en `ur5_tools/` top-level por compatibilidad
> con los imports existentes. Los subpaquetes son namespaces que
> re-exportan los símbolos relevantes. **F11 iter 2** moverá los
> archivos físicamente y dejará shims top-level con `DeprecationWarning`.

## Diagnóstico previo

`ur5_tools` evolucionó como **cajón de sastre**: 49 módulos sin
cohesión visible — un newcomer al codebase tarda en entender qué
módulo pertenece a qué dominio. La auditoría 2026-05-10 lo identificó
como problema #5 (severidad Alto).

## Organización por dominio (F11 iter 1)

```
ur5_tools/
├── __init__.py            (sin re-exports masivos — los consumers
│                           siguen importando directamente desde
│                           los módulos top-level)
│
├── geometry/              ← constantes + math TF + gripper geom
│   └── __init__.py        (re-exporta de ../geometry_constants,
│                           ../tf_geometry_logic, ../gripper_geometry)
│
├── planning/              ← plan_to_pose + trajectory + scene
│   └── __init__.py        (re-exporta de ../plan_to_pose_*,
│                           ../fjt_direct_helpers, etc.)
│
├── gripper/               ← attach + release de objetos
│   └── __init__.py        (re-exporta de ../gripper_attach_*,
│                           ../release_objects_*, ../attach_*)
│
├── state/                 ← system state + evidence + bootstrap
│   └── __init__.py        (re-exporta de ../system_state_manager,
│                           ../evidence_*, ../controller_bootstrap)
│
├── diagnostics/           ← cycle timing + perf + probes
│   └── __init__.py        (re-exporta de ../cycle_*, ../perf_helpers,
│                           ../*_probe, ../jt_smoke_test)
│
├── bridges/               ← bridges Gazebo ↔ ROS
│   └── __init__.py        (sin re-exports automáticos — los nodos
│                           bridges importan rclpy en top-level)
│
├── utils/                 ← utilities transversales
│   └── __init__.py        (re-exporta de ../param_utils,
│                           ../tf_batch_lookups*)
│
└── (49 archivos .py top-level, sin cambios físicos)
```

## Mapeo módulo → dominio

| Dominio | Archivos top-level (sin mover) | LOC |
|---------|--------------------------------|----:|
| **geometry** | `geometry_constants`, `gripper_geometry`, `tf_geometry_logic`, `tf_geometry_service`, `world_tf_publisher`, `object_pose_resolver_service`, `object_pose_cache` | ~1100 |
| **planning** | `plan_to_pose_server`, `plan_to_pose_logic`, `plan_to_pose_helpers`, `plan_to_pose_moveit_direct`, `plan_to_pose_real_bridge`, `fjt_direct_helpers`, `trajectory_executor_contract`, `planning_scene_sync`, `planning_scene_sync_helpers` | ~3200 |
| **gripper** | `gripper_attach_backend`, `gripper_attach_models`, `attach_anchor`, `attach_demo_transport`, `attach_gz_cli`, `attach_math`, `attach_pose_lookup`, `attach_pose_sub`, `attach_set_pose`, `release_objects_service`, `release_objects_geometry`, `release_objects_logic` | ~3500 |
| **state** | `system_state_manager`, `system_health_helpers`, `evidence_logger`, `evidence_helpers`, `controller_bootstrap` | ~1800 |
| **diagnostics** | `cli_cycle_timing`, `clock_probe`, `cycle_logger`, `cycle_timing_aggregator`, `cycle_timing_analyzer`, `generate_latency_table`, `jt_smoke_test`, `perf_helpers`, `tf_probe` | ~1500 |
| **bridges** | `gz_pose_bridge`, `gz_ros_control_guard`, `moveit_bridge_utils` | ~600 |
| **utils** | `param_utils`, `tf_batch_lookups`, `tf_batch_lookups_runtime` | ~400 |
| **TOTAL** | 49 módulos | ~12100 |

## Cómo usar los subpaquetes

**Hoy (F11 iter 1):**

```python
# Imports actuales — siguen funcionando, sin cambios:
from ur5_tools.geometry_constants import BASE_LINK_IN_WORLD
from ur5_tools.plan_to_pose_helpers import select_traj_duration_and_timeout
from ur5_tools.release_objects_geometry import is_pose_on_table

# Imports nuevos via namespace — equivalentes y preferibles:
from ur5_tools.geometry import BASE_LINK_IN_WORLD
from ur5_tools.planning import select_traj_duration_and_timeout
from ur5_tools.gripper import is_pose_on_table
```

Ambas rutas funcionan idénticamente. Los subpaquetes son
**aditivos** — no rompen nada.

## F11 iter 2 (deferred, ~16h)

Mover archivos físicamente:

```
git mv ur5_tools/geometry_constants.py ur5_tools/geometry/
git mv ur5_tools/tf_geometry_logic.py ur5_tools/geometry/
... (~49 movimientos)
```

Cambios necesarios después del move:

1. **Imports relativos**: `from .param_utils import X` →
   `from ..utils.param_utils import X` cuando el origen está dentro
   de un subpaquete.

2. **Entry points**: `setup.py` actualiza
   `"world_tf_publisher = ur5_tools.world_tf_publisher:main"` →
   `"world_tf_publisher = ur5_tools.geometry.world_tf_publisher:main"`.

3. **Shims top-level**: dejar `ur5_tools/world_tf_publisher.py` como
   shim:

   ```python
   import warnings
   warnings.warn(
       "ur5_tools.world_tf_publisher es deprecated; usa "
       "ur5_tools.geometry.world_tf_publisher",
       DeprecationWarning, stacklevel=2,
   )
   from .geometry.world_tf_publisher import *  # noqa
   ```

4. **Tests**: actualizar imports a las nuevas rutas (los shims
   garantizan que tests viejos sigan funcionando durante 1 ciclo).

5. **Documentación**: actualizar `docs/architecture.md`.

**Riesgo iter 2**: ALTO. Cualquier consumer externo (workflows, scripts
fuera del repo) que importe `ur5_tools.foo` directamente requerirá
actualizar a `ur5_tools.{dominio}.foo`. Los shims con `DeprecationWarning`
mitigan pero no eliminan el riesgo.

**Recomendación**: ejecutar iter 2 cuando se migre a robot real (F11
escala con el split de paquetes ROS — uno por dominio, en lugar de
subpaquetes Python).

## F11 iter 3 (deferred, ~24h)

Split en 4 paquetes ROS independientes:

```
src/
├── ur5_planning_tools/    (planning/ + utils/)
├── ur5_gripper_tools/     (gripper/)
├── ur5_state_tools/       (state/ + diagnostics/)
└── ur5_geometry_tools/    (geometry/ + bridges/)
```

Cada uno con su propio `package.xml`, `setup.py`, `entry_points`.
Esto requiere reorganizar `ur5_bringup/launch/` también.

**Riesgo iter 3**: MUY ALTO. Es básicamente reescribir el workspace.
Solo justificable si se publica académicamente o se migra a robot real.
