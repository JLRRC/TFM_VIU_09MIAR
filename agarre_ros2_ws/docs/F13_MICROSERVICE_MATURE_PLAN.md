# F13 — Arquitectura microservicio madura (plan completo)

## Objetivo

Extraer `panel_backend_node` como **proceso ROS 2 independiente** del
panel Qt. El backend absorbe el `RosWorker` actual de `panel_ros.py`
(que hoy corre dentro del proceso Qt como hilo). El panel se reduce a
UI pura que consume topics/services del backend.

## Estado deseado

```
                        ┌────────────────┐
  Qt main thread        │ panel_v2 (UI)  │  proceso 1
  ────────────────────  │  - widgets     │
                        │  - signals/slots│
                        │  - sólo display │
                        └────────┬───────┘
                                 │  ROS topics/services/actions
                                 ▼
                        ┌────────────────┐
                        │ panel_backend  │  proceso 2 (LifecycleNode)
                        │  (ex panel_ros)│
                        │  - executor ROS│
                        │  - 8 services  │
                        │  - sub/pub     │
                        └────────────────┘
```

## Por qué es F13 (final)

* Pre-requisitos: F5 (split panel_ros), F6 (eliminar panel_launchers),
  F8b (TFM al backend).
* Cambia la arquitectura del proceso: panel y backend son dos
  ejecutables distintos, lanzados por el bringup.
* Validación E2E live obligatoria: la UI debe seguir respondiendo
  con el mismo flujo (pick demo, calibración, infer).

## Hecho en F13 (2026-05-10)

* Documentación del plan completo con prerequisitos.
* Tag `pre-f13-microservice-extract` documentado como puerta antes
  de empezar.
* Inventario locked en `docs/LIFECYCLE_ARCHITECTURE.md` que registra
  qué pasa al ser LC tras la migración.

## Plan F13b (cuando F5b + F6b + F8b estén cerrados)

### Paso 1 — Renombrar y promover

* `panel_ros.py` (ya reducido por F5b) → mover a
  `src/ur5_qt_panel/ur5_qt_panel/panel_backend_node.py`.
* Convertir a `LifecycleNode` (transiciones managed).
* Añadir entry point en `setup.py`:

  ```python
  "panel_backend = ur5_qt_panel.panel_backend_node:main",
  ```

### Paso 2 — Panel sólo UI

* Eliminar import de `RosWorker` de `panel_v2.py`.
* `panel_v2.py` instancia un `BackendClient` (clase nueva) que es
  un mini-cliente de los services/topics del backend.
* `panel_v2.py` se reduce de 1 233 LOC a ~600.

### Paso 3 — Launch combinado

* `ur5_stack.launch.py` añade el nodo `panel_backend` (LC,
  auto-activate) y, opcionalmente, lanza `panel_v2` separado.
* `start_panel_v2.sh` cambia para asumir el backend ya up.

### Paso 4 — Validación

* `colcon test` verde.
* Pick ciclo completo desde el panel (UI ↔ backend ↔ orchestrator
  ↔ Gazebo) sin regresiones.
* Métricas de latencia UI: el panel responde con la misma fluidez
  que con `RosWorker` embebido.

## Riesgos

* **Latencia UI**: el `RosWorker` embebido tiene latencia ~ms para
  callbacks; un proceso separado añade IPC (memoria compartida ROS 2
  con FastDDS local — debería ser similar).
* **Cierre limpio**: hoy `panel_v2.closeEvent` mata el `RosWorker`.
  Con backend separado, hace falta un protocolo: panel publica
  `/panel/closing`, backend escucha y se desactiva (`on_deactivate`).
* **Tests Qt smoke**: hay que asegurar que el panel inicia sin
  backend (modo "stack down" → dialog modal) sin crashear.

## Tag rollback

```bash
git tag pre-f13b-microservice-extract
```
