# F9 — Baseline de rendimiento (auditoría 2026-05-04)

Cifras observadas estáticamente en el workspace. Sirven como punto de partida
para profiling dinámico cuando F7+F8 estén cerrados.

## Hot counts

| Métrica | Valor | Lectura |
|---|---|---|
| `time.sleep` en código productivo | 156 | aceptable |
| ↳ en legacy (panel_pick_demo + panel_pick_object) | 36 (23 %) | F8 los elimina |
| ↳ en código no-legacy | 120 | aceptable |
| `QTimer` / `create_timer` en panel | 86 | revisar tras F8 |
| Bridges Gazebo↔ROS registrados | 77 | revisar mínimo necesario |

## Top archivos por `time.sleep` (no-legacy)

| LOC | Archivo | Justificación esperada |
|---|---|---|
| 8 | `panel_motion_helpers.py` | esperar settle del robot — legítimo |
| 6 | `release_objects_service.py` | esperar respuesta GZ SetEntityPose |
| 6 | `panel_tfm_execute.py` | post-execute settle |
| 6 | `panel_gz_objects.py` | spawn/despawn handshake con GZ |
| 6 | `attach_gate_evaluator.py` | gate timing |
| 5 | `moveit_bridge/joint_state_helpers.py` | fresh joint_state poll |
| 5 | `panel_physics.py` | physics step settle |
| 5 | `panel_motion_control.py` | motion stage settle |

Ningún archivo concentra >10 sleeps fuera del legacy. No hay polling de busy-wait.

## Cosas a perfilar live (post-F8)

Cuando F7 desbloquee F8 y se haya estabilizado:

1. **Arranque del stack** — desde `start_panel_v2.sh` hasta `moveit_state=READY`. Objetivo < 12 s.
2. **CPU idle** — panel + orchestrator + bridges + Gazebo en reposo. Objetivo < 60 % cores total.
3. **Latencia ciclo pick_demo** — desde `/pick_place` Goal accept hasta Result. Objetivo < 45 s.
4. **Memory growth** — 30 min de ciclos repetidos. Objetivo leak < 100 MB/h.

Comandos sugeridos:

```bash
# CPU + RSS por nodo durante un ciclo
PICK_E2E_LIVE=1 python3 -m pytest src/ur5_bringup/test/test_e2e_pick_cycles.py -q -s &
sleep 5
top -b -n 30 -d 2 -p $(pgrep -d',' -f 'panel_v2|pick_orchestrator|gz sim|move_group') > report/perf_cycle.txt

# Frecuencia real de cada topic /tf, /joint_states, /scan, etc.
for t in /tf /joint_states /clock; do ros2 topic hz $t & done
wait

# Tracing de timers ROS 2 (cuando F8 cierre)
ros2 trace start
# ... ejecutar 1 ciclo ...
ros2 trace stop
ros2 trace report
```

## Optimizaciones candidatas (ordenadas por riesgo)

| # | Cambio | Riesgo | Ganancia esperada |
|---|---|---|---|
| 1 | Reducir `bridge_cameras.yaml` a topics realmente usados (auditar 77 bridges) | bajo | -10-20 % CPU bridges |
| 2 | Aumentar `update_rate` de joint_state_broadcaster si está a 50 Hz (suficiente 30 Hz para sim) | bajo | -5 % CPU |
| 3 | Reemplazar `time.sleep(0.5)` por `rclpy.spin_once(timeout=...)` en helpers que esperan callbacks | medio | -1-2 s por ciclo |
| 4 | Lazy-load de mixins MoveItPy hasta primer Goal | medio | -3-5 s arranque |
| 5 | Eliminar QTimers redundantes tras F8 (legacy creaba ~20) | bajo (post-F8) | -5 % CPU panel |

## Estado actual

✅ No hay problemas de performance estructurales detectables estáticamente.
⏳ Optimización fina requiere profiling live (post-F8).
