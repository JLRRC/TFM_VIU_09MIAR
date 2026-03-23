# DIRECTO_RELEASE_HOME_VALIDACION_FINAL

## Reproducción real ejecutada
Runner usado:
- `/tmp/run_directo_release_home_once.sh`

Corridas ejecutadas post-parche:
- `RELHOME_RUN4` (válida para tramo final)
- `RELHOME_RUN5` (no válida para tramo final: bloqueo previo de disponibilidad TF/pick)
- `RELHOME_RUN6` (válida para tramo final)

## Evidencia de tramo final (post-parche)
Fuente: `DIRECTO_RELEASE_HOME_TRACE.log`

### RUN4
- `ENTER_PHASE RELEASE`: línea 4
- `mark_object_released=true`: línea 6
- `DETACH publish`: línea 7
- `RELEASE validation=true wait_ok=true mark_ok=true`: línea 9
- `ENTER_PHASE HOME_FINAL`: línea 11
- `HOME_FINAL result=ok` y `EXIT_PHASE HOME_FINAL`: líneas 16-17
- `Secuencia completada`: línea 18

### RUN6
- `ENTER_PHASE RELEASE`: línea 25
- `mark_object_released=true`: línea 27
- `DETACH publish`: línea 28
- `RELEASE validation=true wait_ok=true mark_ok=true`: línea 30
- `ENTER_PHASE HOME_FINAL`: línea 32
- `HOME_FINAL result=ok` y `EXIT_PHASE HOME_FINAL`: líneas 37-38
- `Secuencia completada`: línea 39

## Respuestas obligatorias
1. ¿El objeto se suelta realmente en `RELEASE` o solo lógicamente?
- Se suelta de forma útil para la ruta Directo: detach publicado + estado lógico consolidado (`RELEASED/NONE`) y separación TCP-objeto tras `HOME_FINAL` (`dist_base` 0.253 en RUN4 y 0.149 en RUN6).

2. ¿La validación de release es correcta?
- Sí. Ahora valida con señales explícitas (`wait_ok`, `mark_ok`, `detached`, `owner_none`, `state_ok`) y en corridas válidas queda `validation=true`.

3. ¿El robot entra siempre en `HOME_FINAL`?
- Cuando la corrida llega al tramo final, sí (RUN4 y RUN6). `RUN5` no alcanzó el tramo por bloqueo previo fuera de este frente.

4. ¿El timeout residual en `HOME_FINAL` bloquea la utilidad real de Directo o es solo cierre tardío no crítico?
- No bloquea la utilidad real observada en este frente. En corridas válidas `HOME_FINAL` cerró con `result=ok`.

5. ¿Qué condición exacta falla si Directo no termina limpio?
- Condición dominante previa al parche: lectura prematura/asíncrona de estado tras release (`logical_attached=true`, `owner=ROBOT`, `logical_state=CARRIED`) por dependencia de callback UI para `mark_object_released`.

6. ¿Qué cambio mínimo deja la ruta Directa completa y utilizable fin-a-fin?
- Mover `mark_object_released` al hilo de secuencia + espera corta con reintento de detach/release antes de cerrar `RELEASE`.

## Veredicto técnico del frente final
- `RELEASE`: funcional y limpio en corridas válidas.
- `HOME_FINAL`: funcional (`result=ok`) en corridas válidas.
- Cierre fin-a-fin de este frente: **validado** sobre 2 corridas válidas post-parche.
