# EVIDENCIA TFM MASTER

Fecha: 2026-03-23

## Artefactos que SI sirven como evidencia
- log/ros2_launch.log (traza de arranque, servicios, fases de pick y estado READY).
- auditoria/panel_audit/logs/execute.log (historial de execute TFM con casos OK/FAIL).
- auditoria/panel_audit/logs/infer.log (inferencia, bloqueos y tiempos).
- auditoria/panel_audit/artifacts/grasp_last.json y grasp_rect_last.json (separacion grasp visual vs ejecutable).
- overlays en auditoria/panel_audit/figures (evidencia visual de inferencia).

## Artefactos que NO sirven por si solos
- respuesta success=true de servicio trigger sin traza de fase terminal.
- un unico log parcial de pick_object sin resultado final.
- run con grasp expirado (no prueba ejecucion canonica estable).

## Rutas defendibles hoy
- Arranque/orquestacion del panel: defendible.
- Inference path TFM remoto: defendible tras parche (antes/despues verificable).
- Ruta directa de pick_demo: defendible como ruta secundaria/diagnostica.

## Rutas con limitaciones vigentes
- PickObject MoveIt: no defendible como cierre e2e estable por bloqueo en APPROACH wait_result.
- Ejecutar Agarre canonico: defendible solo con condicion operativa (disparo inmediato tras infer para evitar grasp expirado).

## Contradicciones detectadas y honestidad tecnica
- Sistema puede estar en READY y aun asi no cerrar APPROACH en pick_object.
- Trigger remoto devuelve aceptacion, pero eso no implica exito fisico terminal.
- El cuello actual no es release/freshness (ya mitigado), sino cierre de resultado MoveIt por etapa.

## Recomendacion final para demo/defensa
1. Demo principal: panel_v2 + inferencia TFM remota + evidencia de seleccion/overlay + execute inmediato si grasp fresco.
2. Demo secundaria: pick_demo directo para mostrar ciclo mecanico controlado.
3. No usar pick_object MoveIt como demo principal hasta cerrar APPROACH con resultado terminal repetible.
