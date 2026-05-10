# F9 (continuación) — split del world SDF

## Estado

* **Hecho**: split del bridge YAML monolítico en 4 fragmentos por dominio
  (`scripts/bridges/01_core.yaml` … `04_gripper_anchors.yaml`) +
  `compose.py` regenerador + 7 tests parity.
* **Diferido**: split del world SDF (`worlds/ur5_mesa_objetos.sdf`,
  898 LOC). Razón: extraer cámaras/objetos a `<include>` requiere
  modelos individuales en `models/` con su `model.config + model.sdf`,
  registrar `GZ_SIM_RESOURCE_PATH` y validar spawning live. Coste alto
  + riesgo medio en una sola sesión sin live testing.

## Propuesta para iteración futura

1. Crear `models/picking_objects/<name>/` (un model por objeto).
2. Crear `models/cameras_overhead/`, `models/cameras_lateral/` (uno por
   tipo de cámara — RGB-D vs RGB-only).
3. Sustituir bloques inline en `worlds/ur5_mesa_objetos.sdf` por
   ``<include><uri>model://name</uri><pose>...</pose></include>``.
4. Añadir test ``test_world_models_referenced_exist`` que valide que
   cada `<include>` tiene su modelo correspondiente en `models/`.
5. Validación live: ``ros2 launch ur5_bringup ur5_stack.launch.py
   launch_panel:=false`` debe spawnar todos los objetos con poses
   idénticas a la versión actual.

## Riesgos

* Resource path desincronizado → modelo no encontrado → world no
  spawnea.
* Cambio de pose por reinterpretación de offsets → ciclos pick fallan
  por XY off-by-1cm.

Test E2E live obligatorio antes de cerrar.
