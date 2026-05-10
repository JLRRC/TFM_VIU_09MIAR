# F5 — Split de panel_ros.py (plan completo)

## Contexto

`src/ur5_qt_panel/ur5_qt_panel/panel_ros.py` (2 126 LOC) es el god-file
ROS del panel: incluye 8 services + 5 subscriptions + 4 publishers +
el `RosWorker` core (thread + executor).

## Diagnóstico por dominio

* **Worker thread + executor (~300 LOC)** → `panel_ros_executor.py`.
* **Subscriptions de cámaras (~250 LOC)** → `panel_subscribers_camera.py`.
* **Subscriptions de joints/poses (~300 LOC)** → `panel_subscribers_joint.py`.
* **Subscriptions MoveIt feedback (~200 LOC)** → `panel_subscribers_moveit.py`.
* **Service `/camera_connect|disconnect` (~100 LOC)** → `service_camera.py`.
* **Service `/recover_motion` (~150 LOC)** → `service_recover.py`.
* **Service `/tfm_infer_object|execute_grasp` (~250 LOC)** → `service_tfm_proxy.py`.
* **Service `/pick_demo|pick_object|select_object` (~300 LOC)** →
  `service_pick_proxy.py`.
* **Helpers/utils restantes (~300 LOC)** → quedan en `panel_ros.py` ≤ 800 LOC.

## Por qué se difiere

* Cada extracción cambia el MRO + las variables que comparte el
  `RosWorker`. Un fallo silencioso (e.g. `self._latest_pose` no se
  inicializa) sólo aparece al ejecutar el panel + Gazebo.
* Sin live-test budget, el riesgo de romper el panel es alto.
* El refactor F5 es prerequisito de F13 (microservicio panel_backend),
  por lo que tiene sentido hacerlos juntos cuando haya tiempo.

## Hecho en F5 (2026-05-10)

* Lint `test_panel_ros_loc_baseline.py` que bloquea el crecimiento
  de `panel_ros.py`. Cualquier nueva línea fuerza justificación o
  ejecución del split.
* Plan documentado con la división recomendada por dominios.

## Plan F5b (cuando haya live test budget)

### Estrategia: extracción incremental con re-export

Por cada bloque a extraer:

1. Crear nuevo módulo `panel_<dominio>.py` con la lógica.
2. En `panel_ros.py`, reemplazar el bloque por un import + delegación.
3. Test smoke + offline tests del nuevo módulo.
4. Verificar `ros2 launch ur5_bringup ur5_stack.launch.py` sigue OK.
5. Commit por dominio (8-9 commits).

### Orden recomendado (de menor a mayor riesgo)

1. **service_camera** (más aislado, menos dependencias).
2. **service_recover**.
3. **service_tfm_proxy** (tras F8b, esto se reduce a un pass-through).
4. **service_pick_proxy**.
5. **subscribers_camera, _joint, _moveit** (tocan estado compartido).
6. **panel_ros_executor** (último, es el corazón).

## Riesgos

* `RosWorker.__init__` inicializa estado compartido entre callbacks.
  Si los nuevos módulos no acceden al estado correctamente, hay
  carreras (los callbacks corren en el executor thread, la UI en main).
* Tests offline existentes (`test_panel_v2_*_mixin.py`) podrían
  romperse si el MRO cambia. Mantener los mixins inalterados.

## Tag rollback

```bash
git tag pre-f5b-panel-ros-split
```
