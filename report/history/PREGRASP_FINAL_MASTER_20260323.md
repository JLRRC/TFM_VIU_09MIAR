# Cierre quirurgico PRE_GRASP (estado real)

## Resumen ejecutivo
- Se aplico un parche minimo en el bridge para acotar timeouts efectivos de APPROACH (incluyendo rutas de stall/long-retry).
- Se recompilo `ur5_tools` y se valido en runtime con 3 corridas.
- Resultado: el sistema ya no queda en espera abierta de varios minutos sin cierre; ahora publica/consume resultado terminal de APPROACH de forma determinista.
- Limite actual: PRE_GRASP no inicia porque APPROACH sigue terminando en `exec_failed_fjt_direct:fjt_result_timeout`.

## Evidencia clave
- Corrida 1:
  - `APPROACH timeout capped original=157.9 max=35.0`
  - `DISPATCH_END request_id=1 elapsed_us=49,877,558 success=False plan_ok=True exec_ok=False`
  - `RESULT_PUBLISHED request_id=1`
  - Panel consume: `RESULT_DIAGNOSTIC ... request_id=1 ... delta_us=50,143,462`
- Corrida 2:
  - Capping observado: `original=172.8` y `original=258.6` (ambos cap a 35.0)
  - `DISPATCH_END request_id=2 elapsed_us=86,714,912 success=False`
  - `RESULT_PUBLISHED request_id=2`
  - Panel consume: `RESULT_DIAGNOSTIC ... request_id=2 ... delta_us=87,013,387`
- Corrida 3:
  - `APPROACH timeout capped original=162.8 max=35.0`
  - `DISPATCH_END request_id=3 elapsed_us=49,831,215 success=False`
  - `RESULT_PUBLISHED request_id=3`
  - Panel consume: `RESULT_DIAGNOSTIC ... request_id=3 ... delta_us=50,126,425`

## Causa raiz unica (PRE_GRASP bloqueado)
- PRE_GRASP no se ejecuta porque la fase previa APPROACH no llega a exito.
- El bloqueo dominante previo (esperas largas/no deterministas por timeouts inflados en APPROACH) quedo contenido con el parche.
- Queda un fallo funcional de ejecucion FJT en APPROACH (timeout de ejecucion), no de enrutado request/result.

## Parche minimo aplicado
- Archivo: `src/ur5_tools/ur5_tools/ur5_moveit_bridge.py`
- Cambio: reutilizar `PANEL_MOVEIT_BRIDGE_APPROACH_MAX_TOTAL_TIMEOUT_SEC` (35s por defecto) para:
  - cap del timeout efectivo tras `prepared_timeout`
  - cap en retry timeout de rutas `approach_stall_retry` y `approach_long_retry`
  - consolidacion del cap existente en goal_time_tolerance retry

## Veredicto
- Estabilidad del flujo request/result: SI (3/3 con publish+consume consistente)
- Entrada a PRE_GRASP: NO
- Cierre quirurgico PRE_GRASP (funcional): PENDIENTE (depende de resolver timeout FJT en APPROACH)
