# GitHub Actions

F3 (auditoría 2026-05-10).

## Workflows

### `ci.yml` — CI principal

Disparadores: `push` a `main` o ramas `refactor/**`, `pull_request` a `main`,
o ejecución manual (`workflow_dispatch`).

Jobs:

1. **lint**: `flake8` + `pydocstyle` (PEP 257) sobre los paquetes
   Python. PEP 257 sólo enforced en módulos puros del orchestrator y
   `geometry_constants` (los paquetes con god-modules pendientes de
   refactor — F6/F9 — se irán incorporando).

2. **pytest-offline**: tests que pueden ejecutarse sin instalación ROS:
   - `ur5_tools/test_geometry_constants.py` y `test_gripper_geometry.py`
   - Módulos puros de `tfm_orchestrator` (FSM, gates, retry, etc.)
   - Smoke tests del panel Qt (`test_imports_qt_panel`,
     `test_quality_metrics`).

   Los tests `launch_testing` y los que requieren `rclpy` real **no se
   ejecutan en este job** — son demasiado costosos sin Gazebo y se
   añadirán al workflow `colcon-test.yml` (F11 cuando reduzcamos deuda).

3. **policy-checks**: validaciones rápidas que pueden bloquear PRs:
   - **No Gazebo Classic**: cualquier referencia a `gazebo_ros`,
     `libgazebo_ros_control` o `gazebo_ros_pkgs` falla el job. El
     proyecto usa exclusivamente `gz_ros2_control` (Gazebo Harmonic).
   - **No `BASE_LINK_IN_WORLD` duplicado**: aviso si alguien
     reintroduce el literal `(-0.85, 0, 0.85)` hardcoded en código
     fuente productivo. El test AST en
     `ur5_tools/test/test_geometry_constants.py` es el enforce real.

## Pendiente (futuras fases)

- **F11 colcon-test.yml**: build + test completo con ROS 2 Jazzy real
  vía `ros-tooling/setup-ros@v0.7`. Requiere reducir deuda de tests
  `launch_testing` que aún no son self-contained.
- **F4 lint enforcement**: pasar de `advisory` a `failing` cuando los
  4 paquetes CMake (ur5_bringup, ur5_description, ur5_moveit_config,
  ur5_panel_interfaces) tengan `ament_flake8` y `ament_pep257` en sus
  `package.xml`. Ahora mismo el lint job es report-only para no
  bloquear merges con la deuda existente.
