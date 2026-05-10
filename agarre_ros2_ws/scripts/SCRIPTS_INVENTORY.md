# Scripts inventory — `agarre_ros2_ws/scripts/`

**Fecha:** 2026-05-10 (auditoría iteración 2 — F-iter2 fix-3).

Catalogación de los **~75 scripts** en `scripts/` por propósito y estado. El objetivo es facilitar onboarding y eliminar friction: cualquier script no listado aquí necesita ser categorizado.

---

## 1. Instalación / arranque

| Script | Propósito | Estado |
|---|---|---|
| `01_install_ros2_jazzy.sh` | Instala ROS 2 Jazzy desde apt | **VIVO** |
| `01_preflight_check.sh` | Smoke check pre-lanzamiento (services + topics) | **VIVO** |
| `01_run_all_preflight.sh` | Wrapper que ejecuta todos los `01_*` | **VIVO** |
| `01_validate_installation.sh` | Valida que el WS está bien construido | **VIVO** |
| `fresh_build.sh` | colcon build clean | **VIVO** |
| `start_panel_v2.sh` | Lanza el panel Qt | **VIVO** |
| `start_panel_v2_venv.sh` | Variante con venv | **VIVO** |
| `start_panel_gui.sh` | Variante GUI (con DISPLAY) | **VIVO** |
| `start_panel_with_xvfb.sh` | Variante headless con Xvfb (CI) | **VIVO** |
| `stop_panel_v2.sh` | Mata el panel + procesos asociados | **VIVO** |
| `status_panel_v2.sh` | Muestra estado de procesos | **VIVO** |
| `recover_panel_v2.sh` | Recovery tras crash | **VIVO** |
| `kill_all.sh` | Kill agresivo de todos los procesos ROS | **VIVO** |
| `limpia_stack.sh` | Cleanup post-sesión | **VIVO** |

---

## 2. Validación / smoke tests

| Script | Propósito | Estado |
|---|---|---|
| `validate_pick_3_cycles.sh` | 3 ciclos pick (F7 audit, KPI JSON) | **CANÓNICO** |
| `validate_before_demo.sh` | Pre-demo checklist | **VIVO** |
| `validate_panel_flow.sh` | Smoke del flujo del panel | **VIVO** |
| `validate_startup_repro.sh` | Reproducibilidad de arranque | **VIVO** |
| `validate_touch_dual.sh` | Validación dual-finger touch | **VIVO** |
| `panel_block_smoke_test.sh` | Smoke del panel Qt (offscreen) | **VIVO** |
| `smoke_test.sh` | Smoke wrapper general | **VIVO** |
| `tfm_smoketest.py` | Smoke del módulo TFM | **VIVO** |
| `evidence_startup_ready.sh` | Captura evidence al arrancar | **VIVO** |
| `repro_release_objects_idempotent.sh` | Test regresión release idempotente | **VIVO** |
| `test_geometry_regression.sh` | Regresión geométrica | **VIVO** |
| `test_moveit_regression.sh` | Regresión MoveIt | **VIVO** |
| `test_pick_physics_regression.sh` | Regresión física pick | **VIVO** |
| `run_tfm_regression_suite.sh` | Suite completa de regresión | **CANÓNICO** |

---

## 3. Diagnóstico / debug

| Script | Propósito | Estado |
|---|---|---|
| `diag_gazebo_camera_ctrl.sh` | Debug de cámaras Gazebo | **VIVO** |
| `diag_startup_health.sh` | Healthcheck startup | **VIVO** |
| `diag_tf_tcp.sh` | Debug TF chain del TCP | **VIVO** |
| `debug_bridge_isolated.sh` | Debug bridge ros_gz aislado | **VIVO** |
| `debug_grasp_geometry.py` | Debug geometría de grasp | **VIVO** |
| `monitor_attach_publishers.sh` | Monitorea publishers del attach | **VIVO** |
| `monitor_tip_table_clearance.py` | Monitorea distancia TCP-mesa | **VIVO** |
| `audit_moveit2_system.py` | Audit del sistema MoveIt 2 | **VIVO** |
| `verify_object_state_invariants.py` | Invariantes object state | **VIVO** |
| `check_qt_env.sh` | Check entorno Qt | **VIVO** |

---

## 4. Pick / E2E ejecución

| Script | Propósito | Estado |
|---|---|---|
| `run_pick_cycles_e2e.py` | E2E ciclos pick (Python) | **CANÓNICO** |
| `run_single_pick_pickdemo.py` | Single pick demo | **VIVO** |
| `run_panel_objectstate_check.sh` | Check object state del panel | **VIVO** |
| `run_directo_visual_smoke.sh` | Smoke visual DIRECTO | **VIVO** |
| `run_moveit_lift_validation.sh` | Validación lift con MoveIt | **VIVO** |
| `run_touch_tuner_front_left.sh` | Tuner de touch front-left | **VIVO** |
| `run_touch_tuner_front_right.sh` | Tuner de touch front-right | **VIVO** |
| `freeze_touch_baseline.sh` | Congela baseline de touch | **VIVO** |

---

## 5. Performance / benchmarks

| Script | Propósito | Estado |
|---|---|---|
| `panel_perf_measure.py` | Medición performance panel | **VIVO** |
| `panel_perf_compare.py` | Compara baseline vs current | **VIVO** |
| `bench_infer_log.py` | Benchmark de inferencia | **VIVO** |
| `grasp_audit_benchmark.py` | Benchmark del audit de grasps | **VIVO** |
| `grasp_audit_trace_capture.py` | Captura trace de audit | **VIVO** |
| `summarize_directo_batch.py` | Resume batch DIRECTO | **VIVO** |

---

## 6. Evidencia / reportes

| Script | Propósito | Estado |
|---|---|---|
| `export_tfm_evidence.py` | Exporta evidencia TFM | **VIVO** |
| `show_latest_startup_evidence.sh` | Muestra última evidencia | **VIVO** |
| `prune_startup_evidence.sh` | Limpia evidencia antigua | **VIVO** |
| `generar_base_conocimiento_tfm.sh` | Genera base conocimiento TFM (Markdown) | **VIVO** |
| `capture_camera_frames.py` | Captura frames de cámara | **VIVO** |
| `capture_system_diag.py` | Captura diagnostic del sistema | **VIVO** |
| `directo_visual_capture.py` | Captura visual DIRECTO | **VIVO** |

---

## 7. Tooling / utilidades

| Script | Propósito | Estado |
|---|---|---|
| `prepare_runtime.py` | F8 audit — prepara artifacts pre-launch | **CANÓNICO** |
| `sync_object_positions_to_sdf.py` | Sincroniza posiciones objeto → SDF | **VIVO** |
| `gen_drop_grid.py` | Genera grid de drop poses | **VIVO** |
| `fase_b_validate.py` | Validación fase B (legacy audit) | **LEGACY** |
| `ros2_bringup_checklist.sh` | Checklist bringup | **VIVO** |
| `ci_local.sh` | CI local (mimic GitHub Actions) | **VIVO** |

---

## 8. Datos / configuración

| Archivo | Propósito | Estado |
|---|---|---|
| `fastdds_no_shm.xml` | Profile FastDDS sin SHM | **CONFIG** |
| `panel_runtime_validated.env` | Env vars validadas | **CONFIG** |
| `ur5_home_pose.env` | Pose HOME en formato env | **CONFIG** |
| `object_positions.json` | Posiciones canónicas objetos | **CONFIG** |
| `table_pixel_map.json` | Mapeo pixel → metros (cámara cenital) | **CONFIG** |

---

## 9. Estados

- **CANÓNICO**: el script principal de su categoría (referenciado en docs).
- **VIVO**: script en uso, mantenido.
- **CONFIG**: archivos de configuración (no ejecutables).
- **LEGACY**: deprecado pero no eliminado por safety; revisar en iteración futura.

---

## 10. Reglas de mantenimiento

1. **Cualquier script nuevo debe añadirse a este inventario** en el commit que lo crea.
2. Si un script queda sin uso por >3 meses → marcarlo `LEGACY`.
3. Si está `LEGACY` por >6 meses → considerar `git rm` + commit de cleanup.
4. Los scripts `CANÓNICO` deben tener tests automatizados (`scripts/test_*.sh` + entry en CI).

---

## 11. Pendientes (iteración 3+)

- [ ] Subdividir `scripts/` en `scripts/{install,validate,debug,run,bench,evidence,tooling,config}/` para reducir clutter visual.
- [ ] Añadir shebang + `chmod +x` consistente a todos los `.sh`.
- [ ] Migrar wrappers bash (`validate_pick_3_cycles.sh`, etc.) a Python para portabilidad Windows.
- [ ] Eliminar `fase_b_validate.py` si se confirma legacy en iter 3.
