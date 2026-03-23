# DIRECTO_RELEASE_HOME_FIX

## Alcance aplicado
Cierre exclusivo del tramo `CARRY -> RELEASE -> HOME_FINAL` en ruta **Agarre Objeto (Directo)**, sin tocar MoveIt ni reabrir frentes previos.

Archivo modificado:
- `agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_pick_demo.py`

## Causa raíz dominante
En `short_release_mode`, la suelta lógica dependía de `panel.signal_run_ui.emit(...)` para ejecutar `mark_object_released(...)` dentro de callback UI. Esa ejecución era asíncrona y no determinista respecto al hilo de la secuencia Directo, por lo que la validación inmediata de `RELEASE` podía leer todavía `logical_attached=true`, dejando `RELEASE` en warning e inconsistencia al entrar a `HOME_FINAL`.

## Cambio mínimo aplicado
En el bloque `short_release_mode`:
1. Separé apertura de pinza (UI) de actualización lógica de release.
2. Ejecuté `mark_object_released(..., reason="demo_short_release_worker")` desde el hilo de la secuencia (determinista).
3. Añadí espera corta de confirmación de release con timeout configurable (`PANEL_PICK_DEMO_RELEASE_WAIT_SEC`, por defecto `1.6s`).
4. Añadí reintento único de detach + release lógico al borde del timeout (`demo_short_release_retry`).
5. Conservé y amplié trazas de `RELEASE` y `HOME_FINAL` para dejar evidencia de decisión (`wait_ok`, `mark_ok`, `validation`).

Zona de código:
- `panel_pick_demo.py:2023-2151`

## Resultado esperado del parche
- `RELEASE` deja de depender de latencia/event-loop UI para marcar estado lógico.
- La transición a `HOME_FINAL` entra con estado de release consolidado (`logical_attached=false`, `owner=NONE`, `logical_state=RELEASED`).
- El final de ciclo queda utilizable aun si hay inestabilidad previa fuera de este tramo.
