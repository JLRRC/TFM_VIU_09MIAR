# evidence_logger — F10

Nodo ROS 2 que graba eventos del pipeline pick & place a disco para
reproducibilidad y defensa académica.

## Lanzar

```bash
# Output bajo ${WS_DIR}/report/runs/<timestamp>/ (default).
ros2 run ur5_tools evidence_logger

# Override del directorio raíz:
ros2 run ur5_tools evidence_logger --ros-args -p output_root:=/tmp/runs
```

## Outputs por sesión

Cada arranque crea `report/runs/<YYYYMMDD_HHMMSS>/` con 3 ficheros:

### `events.jsonl` — JSON Lines, una línea por evento

```json
{"ts_iso":"2026-05-01T07:30:00.123456Z","ts_mono":1234.56,"ts_sim_ns":1714540200000000000,"kind":"grasp_result","data":{"raw":"success=true reason=exec_ok","success":true,"reason":"exec_ok"}}
```

| Campo | Tipo | Descripción |
|---|---|---|
| `ts_iso` | str | Timestamp ISO 8601 UTC (microsegundos) |
| `ts_mono` | float | `time.monotonic()` del proceso del logger |
| `ts_sim_ns` | int | Reloj ROS (sim si `use_sim_time:=true`) en ns |
| `kind` | str | Tipo de evento (ver tabla abajo) |
| `data` | dict | Payload específico del kind |

### Tipos de evento (`kind`)

| kind | Origen | data fields |
|---|---|---|
| `session_started` | logger init | `session_dir`, `events_path`, `summary_path` |
| `session_finished` | logger shutdown | `{}` |
| `grasp_result` | `/desired_grasp/result` | `raw`, `success` (bool/None), `reason` (str) |
| `system_state` | `/system_state` | `raw` (str) |
| `system_diag` | `/system_diag` | `raw` (str) |
| `gripper_state` | `/gripper/<obj>/state` | `object` (str), `attached` (bool) |

### `summary.csv` — agregado humano-friendly

```csv
ts_iso,kind,object,success,reason
2026-05-01T07:30:00.123456Z,grasp_result,,true,exec_ok
2026-05-01T07:30:01.987654Z,gripper_state,box_red,true,
```

### `metrics.json` — agregados por sesión (escrito en shutdown)

```json
{
  "total_events": 142,
  "by_kind": {"grasp_result": 12, "gripper_state": 8, "system_diag": 122},
  "grasp_success": 10,
  "grasp_failure": 2,
  "grasp_unknown": 0,
  "grasp_success_rate": 0.8333,
  "attach_count": 4,
  "detach_count": 4,
  "objects_attached": ["box_red", "cyl_blue"],
  "session_started_iso": "2026-05-01T07:00:00.000000Z",
  "session_finished_iso": "2026-05-01T07:15:30.123456Z",
  "duration_sec": 930.12,
  "session_dir": "/path/to/report/runs/20260501_070000"
}
```

## Schema garantizado por tests

Los helpers puros (`parse_grasp_result`, `compute_session_metrics`)
están en `ur5_tools/evidence_helpers.py` y cubiertos por
`test/test_evidence_logger_offline.py` (20 unit tests, sin ROS). El
schema documentado arriba es exactamente lo que esos tests verifican.

## Uso para defensa académica

- Un ciclo de pick&place por línea en `summary.csv` permite tabular
  resultados directamente en el LaTeX del TFM:
  ```python
  import pandas as pd
  df = pd.read_csv("report/runs/<stamp>/summary.csv")
  df[df.kind == "grasp_result"]["success"].value_counts()
  ```
- `metrics.json` da el resumen ejecutivo: tasa de éxito, número de
  attaches, duración total.
- `events.jsonl` es la fuente canónica para análisis post-hoc:
  reconstrucción de timing, latencias entre fases, debug profundo.
