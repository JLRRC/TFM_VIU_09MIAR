# Jerarquía de configuración — env > YAML > defaults

> Última actualización: 2026-05-01 (F11).

## 1. Regla de precedencia

Para cualquier parámetro tunable del sistema, la resolución se hace en este orden:

1. **Variable de entorno** (`os.environ.get("PARAM_NAME")`) — mayor precedencia.
2. **YAML del paquete** (`runtime_defaults.yaml`, `panel_*_runtime.yaml`).
3. **Default literal en código** — fallback si las dos anteriores faltan.

```python
def read_param(name: str, default):
    env_val = os.environ.get(name)
    if env_val is not None:
        return _coerce(env_val, type(default))
    yaml_val = _yaml_lookup(name)
    if yaml_val is not None:
        return yaml_val
    return default
```

## 2. Cuándo usar cada nivel

| Nivel | Uso correcto | Ejemplo |
|---|---|---|
| **Env var** | Override puntual de debug, experimento ad-hoc, CI tweak | `PICK_DEBUG=1`, `STRICT_SELF_COLLISION=1`, `PANEL_NO_GZ=1` |
| **YAML** | Configuración de despliegue estable, valores que deberían persistir | `controller_ready_timeout_sec: 20.0`, `pose_info_max_age_sec: 1.0` |
| **Default literal** | Sentinel mínimo para que el nodo arranque sin YAML ni env | `default=0.5` en `read_param` |

## 3. Anti-patrones a evitar

- ❌ Hardcoded en código sin posibilidad de override (`timeout = 5.0`).
- ❌ Env var sin entrada equivalente en YAML — pierde trazabilidad.
- ❌ Mismo parámetro en dos YAMLs distintos (drift garantizado).
- ❌ Lectura de env var fuera de la capa de configuración (en lógica de negocio).

## 4. YAMLs canónicos por dominio

| Dominio | YAML | Paquete |
|---|---|---|
| Sistema/paths globales | `runtime_defaults.yaml` | `ur5_bringup/config/` |
| Panel UI runtime | `panel_ui_runtime.yaml` | `ur5_qt_panel/config/` |
| Panel ROS layer | `panel_ros_runtime.yaml` | `ur5_qt_panel/config/` |
| Panel pick_demo | `pick_demo_runtime.yaml` | `ur5_qt_panel/config/` |
| Panel pick_object | `pick_object_runtime.yaml` | `ur5_qt_panel/config/` |
| Panel TFM (inferencia) | `panel_tfm_runtime.yaml` | `ur5_qt_panel/config/` |
| Panel settings (compat) | `panel_settings.yaml` | `ur5_qt_panel/config/` |
| Test tuning | `panel_test_tuning.yaml` | `ur5_qt_panel/config/` |
| MoveIt bridge | `moveit_bridge_runtime.yaml` | `ur5_tools/config/` |
| system_state_manager | `system_state_manager.yaml` | `ur5_bringup/config/` |
| Controllers ros2_control | `ur5_mock_controllers.yaml` | `ur5_bringup/config/` |
| Controllers HW | `ur5_controllers.yaml` | `ur5_description/config/` |
| MoveIt config | `kinematics.yaml`, `joint_limits.yaml`, `moveit_controllers.yaml`, `ompl_planning.yaml`, `planning_scene_monitor_parameters.yaml` | `ur5_moveit_config/config/` |

## 5. Estado actual

- **133 `os.environ.get(...)`** restantes en `ur5_qt_panel` (reducido desde ~600 en F2).
- Cada env var debería tener entrada equivalente en alguno de los YAMLs canónicos.
- Tarea pendiente F18: test `test_yaml_consistency.py` que verifique drift env↔YAML.

## 6. Variables de entorno con uso especial

| Env var | Significado | Default |
|---|---|---|
| `STRICT_SELF_COLLISION=1` | Carga `ur5_strict.srdf` en lugar de `ur5.srdf` | 0 |
| `USE_LEGACY_PICK_DEMO=1` | (F12) Mantener `run_pick_demo` legacy en lugar de orchestrator | 0 |
| `PANEL_NO_GZ=1` | Arranca panel sin Gazebo (debug UI) | 0 |
| `PICK_DEBUG=1` | Verbosidad extendida de fases pick | 0 |
| `WS_DIR` | Workspace path override | `agarre_ros2_ws` autodetect |
