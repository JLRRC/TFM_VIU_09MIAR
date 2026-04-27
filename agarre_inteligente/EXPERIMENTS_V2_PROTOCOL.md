# Protocolo de separación de salidas — ENTREGA.V2

## Principio fundamental

El árbol `report/` en la raíz del proyecto contiene evidencia oficial del TFM
presentado. No debe modificarse bajo ningún concepto en trabajos posteriores.

## Dónde van las salidas nuevas

| Tipo de salida | Directorio |
|---|---|
| Logs operativos y auditoría del panel | `auditoria/panel_audit/` (excluido de git) |
| Campañas de validación y debug | `auditoria/` (excluido de git) |
| Nuevos experimentos de entrenamiento | `agarre_inteligente/experiments/EXP5_*` o superior |
| Evidencia de runs de reproducción | `agarre_ros2_ws/report/repro_startup/` (excluido de git) |
| Resultados trackeados de V2 en adelante | `reports_v2/` (si se crea, debe documentarse explícitamente) |

## Scripts que NO deben ejecutarse en ENTREGA.V2 sin control explícito

Los siguientes scripts escriben directamente en `report/` y pueden contaminar
la evidencia oficial si se ejecutan sin cambiar previamente sus rutas de salida:

- `recrear_artefactos_tfm.sh`
- `recrear_experimentos_cap5_gpu.sh`

Si se necesita regenerar artefactos para comprobación, primero redirigir la
salida a un directorio nuevo (p.ej. `reports_v2/`) y verificar que el resultado
es idéntico antes de cualquier comparación.

## Convención de nombres para experimentos nuevos

Los experimentos EXP1–EXP4 son oficiales del TFM y sus checkpoints son inmutables.
EXP1.1 y EXP1.2 son auxiliares del retrain posterior a la defensa.

Cualquier experimento nuevo debe usar:
- `EXP5_*`, `EXP6_*`, etc. — para experimentos nuevos de entrenamiento

Nunca sobreescribir los directorios EXP1..EXP4 ni EXP1.1/EXP1.2.

## Referencia de integridad de report/

- Commit de referencia: `dacace8`
- Fingerprint MD5 del árbol report/ en el momento de crear ENTREGA.V2:
  `e06c4c1abd0094b674023470f9cf1f84`
- Verificar integridad: `find ~/TFM/report -type f | sort | xargs md5sum | md5sum`
