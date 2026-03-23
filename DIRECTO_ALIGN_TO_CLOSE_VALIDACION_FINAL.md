# DIRECTO_ALIGN_TO_CLOSE_VALIDACION_FINAL

## Revalidación real ejecutada
Se ejecutaron corridas reales de la ruta Directo con trigger ROS (`/panel/select_object` + `/panel/pick_demo`) y trazas activas.

Corridas usadas para validación final tras parche:
- RUN5
- RUN6

## Evidencia de transición (post-parche)

### RUN5
- Sale de `GRASP_ALIGN_IK`:
  - `/home/laboratorio/TFM/DIRECTO_ALIGN_TO_CLOSE_TRACE.log:132`
- Entra en `PRE_CLOSE`:
  - `/home/laboratorio/TFM/DIRECTO_ALIGN_TO_CLOSE_TRACE.log:137`
- Entra en `CLOSE`:
  - `/home/laboratorio/TFM/DIRECTO_ALIGN_TO_CLOSE_TRACE.log:148`
- Entra en `ATTACH_GATE`:
  - `/home/laboratorio/TFM/DIRECTO_ALIGN_TO_CLOSE_TRACE.log:155`
- Cierre confirmado en ventana correcta:
  - `/home/laboratorio/TFM/DIRECTO_ALIGN_TO_CLOSE_TRACE.log:154`

### RUN6
- Sale de `GRASP_ALIGN_IK`:
  - `/home/laboratorio/TFM/DIRECTO_ALIGN_TO_CLOSE_TRACE.log:198`
- Entra en `PRE_CLOSE`:
  - `/home/laboratorio/TFM/DIRECTO_ALIGN_TO_CLOSE_TRACE.log:203`
- Entra en `CLOSE`:
  - `/home/laboratorio/TFM/DIRECTO_ALIGN_TO_CLOSE_TRACE.log:214`
- Entra en `ATTACH_GATE`:
  - `/home/laboratorio/TFM/DIRECTO_ALIGN_TO_CLOSE_TRACE.log:221`
- Cierre confirmado en ventana correcta:
  - `/home/laboratorio/TFM/DIRECTO_ALIGN_TO_CLOSE_TRACE.log:220`

## Verificación de síntoma original
Síntoma reportado: se quedaba en alineación o cerraba mal/muy tarde.

Resultado post-parche en 2/2 corridas de validación:
- transición `GRASP_ALIGN_IK -> PRE_CLOSE -> CLOSE -> ATTACH_GATE` completada,
- cierre ejecutado cuando corresponde,
- sin bloqueo en el tramo objetivo.

## Observación fuera de alcance
En RUN6 aparece fallo posterior en `CARRY` (`demo_carry_validation_failed`), posterior a `ATTACH_GATE` y fuera del frente pedido.
- Evidencia:
  - `/home/laboratorio/TFM/DIRECTO_ALIGN_TO_CLOSE_TRACE.log:242`
