# F6 — Mover launches al bringup (plan completo)

## Contexto del problema

`src/ur5_qt_panel/ur5_qt_panel/panel_launchers.py` (1 033 LOC)
arranca Gazebo / MoveIt / RViz / bridges desde el panel Qt usando
`subprocess.Popen`. Esto convierte la UI en el orquestador del stack
ROS 2: incumple separación de responsabilidades.

`panel_gz_startup.py` (826 LOC) y `panel_launch_control.py` (89 LOC)
también participan: spawn entities, bring-up Gazebo, kill processes.

Total acoplado: ~1 950 LOC en el panel para arranque del stack.

## Estado deseado

* `ros2 launch ur5_bringup ur5_stack.launch.py launch_panel:=false`
  arranca todo el sistema (Gazebo + MoveIt + bridges + nodos auxiliares).
* El panel se ejecuta **después** o por separado y se suscribe a
  `/system_state`. Si != READY, muestra un dialog "stack down — usa
  `ros2 launch ur5_bringup ur5_stack.launch.py`".
* `panel_launchers.py` se vacía hasta ~50 LOC (sólo health-check de
  servicios externos). `panel_gz_startup.py` se elimina.

## Por qué se difiere

* Migración requiere validación E2E live: arrancar el stack desde
  `ros2 launch`, conectar el panel y verificar que todos los flujos
  funcionan (pick demo, calibración, TFM inference).
* `panel_v2.py` instancia `panel_launchers` en su `__init__`; si la
  inicialización cambia, el MRO de mixins puede romperse.
* Sin live-test budget, el riesgo es romper el arranque del usuario.

## Hecho en F6 (2026-05-10)

* Lint `test_panel_launchers_loc_baseline.py` que bloquea el
  crecimiento de `panel_launchers.py` y `panel_gz_startup.py`. Sirve
  como puerta: cualquier nuevo código de launch en el panel hace
  fallar el test.
* Documentación de plan de migración y validación.

## Plan F6b (cuando haya live test budget)

### Paso 1 — Inventario de procesos lanzados

* Ejecutar `panel_v2` con `PANEL_AUDIT_LAUNCH=1` (env nueva) que loga
  cada `subprocess.Popen` con su `cmd`, `pid` y `cwd`.
* Cruzar con `ur5_stack.launch.py` para ver qué ya está cubierto.

### Paso 2 — Migrar uno a uno

Por cada proceso lanzado en `panel_launchers.py`:
1. Verificar que `ur5_stack.launch.py` o un sub-launch lo arranca
   (la mayoría sí — Gazebo, MoveIt, bridges, controllers).
2. Marcar el método del panel como `@deprecated` + raise warning.
3. Cuando todos están migrados, eliminar el método.

### Paso 3 — Health check del stack

* `panel_v2` arranca → suscribe `/system_state` (existe ya con LC).
* Si después de 10 s no llega `READY`, mostrar dialog modal.
* Eliminar `panel_launchers.py` y `panel_gz_startup.py` (LOC bajan
  ~1 900).

### Paso 4 — Validación

* `ros2 launch ur5_bringup ur5_stack.launch.py launch_panel:=false`
  + `ros2 run ur5_qt_panel panel_v2` debe replicar el comportamiento
  actual (`pick_demo` ciclo completo).
* `colcon test` verde.

## Riesgos

* **Quitar `panel_launchers.py` rompe `panel_v2.py`** si los mixins
  llaman a sus métodos. Requiere análisis del MRO.
* **Variables de entorno** que el panel inyectaba a los procesos
  (e.g. `GZ_PARTITION`, `QT_QPA_PLATFORM`) deben estar en
  `runtime_defaults.yaml` y propagarse vía `SetEnvironmentVariable`
  en el launch.

## Tag rollback

```bash
git tag pre-f6b-launches-migration
```
