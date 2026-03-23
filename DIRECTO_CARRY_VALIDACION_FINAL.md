# DIRECTO_CARRY_VALIDACION_FINAL

## Reproducción y revalidación real

Corridas usadas:
- `CARRY_BASE_1` (baseline fallida)
- `CARRY_FIX_1R` (post-parche)
- `CARRY_FIX_2R` (post-parche)

### Baseline (fallo reproducido)
- Entra en `ATTACH_GATE`, `LIFT`, `CARRY`.
- Falla con `CARRY_NOT_ACQUIRED` / `demo_carry_validation_failed`.
- Señal clave: `obj_lift_delta=0.024` (< `min_lift_delta=0.025`) y degradación posterior.
- Evidencia:
  - `/home/laboratorio/TFM/DIRECTO_CARRY_TRACE.log:2`
  - `/home/laboratorio/TFM/DIRECTO_CARRY_TRACE.log:10`
  - `/home/laboratorio/TFM/DIRECTO_CARRY_TRACE.log:16`
  - `/home/laboratorio/TFM/DIRECTO_CARRY_TRACE.log:22`

### Revalidación 1 (post-parche)
- `post_attach_hold wait_sec=0.90` visible.
- `LIFT` con `obj_lift_delta=0.065`.
- `CARRY` validado (`cond_obj_move=true`, `cond_lift=true`, `cond_tcp=true`).
- Sin `demo_carry_validation_failed`.
- Evidencia:
  - `/home/laboratorio/TFM/DIRECTO_CARRY_TRACE.log:31`
  - `/home/laboratorio/TFM/DIRECTO_CARRY_TRACE.log:34`
  - `/home/laboratorio/TFM/DIRECTO_CARRY_TRACE.log:38`
  - `/home/laboratorio/TFM/DIRECTO_CARRY_TRACE.log:40`

### Revalidación 2 (post-parche)
- `post_attach_hold wait_sec=0.90` visible.
- `LIFT` con `obj_lift_delta=0.065`.
- `CARRY` validado.
- Sin `demo_carry_validation_failed`.
- Evidencia:
  - `/home/laboratorio/TFM/DIRECTO_CARRY_TRACE.log:49`
  - `/home/laboratorio/TFM/DIRECTO_CARRY_TRACE.log:52`
  - `/home/laboratorio/TFM/DIRECTO_CARRY_TRACE.log:56`
  - `/home/laboratorio/TFM/DIRECTO_CARRY_TRACE.log:58`

## Conclusión de este frente
- El fallo dominante post-attach quedó controlado en el tramo objetivo.
- La ruta pasa `ATTACH_GATE -> LIFT -> CARRY` en 2/2 corridas post-parche.
- El siguiente cuello, si aparece, queda fuera de este frente (release/home final).

