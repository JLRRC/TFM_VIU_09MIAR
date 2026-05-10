# Pre-flight checklist robot real — UR5 CB-series + OnRobot RG2

**Fecha:** 2026-05-10 (auditoría F10.4).

Este documento es la guía de pre-flight para **migrar de simulación Gazebo a hardware real** UR5 (CB3 con `ur_robot_driver` o equivalente) + OnRobot RG2 físico.

> ⚠️ **IMPORTANTE:** este documento describe el plan; la validación efectiva con UR5 real **no se ha ejecutado aún**. Es un checklist accionable cuando se disponga del hardware.

---

## 0. Pre-requisitos hardware

- [ ] UR5 CB3 con cable Ethernet conectado y configurado en URCap.
- [ ] OnRobot RG2 montado físicamente en flange + conectado a controlador UR.
- [ ] PC con Ubuntu 24.04 + ROS 2 Jazzy + workspace TFM compilado.
- [ ] Red local con IP estática para el UR (default 192.168.1.102).
- [ ] Botón emergency stop accesible.
- [ ] Mesa y objetos físicos coincidentes con la pose declarada en `geometry.yaml`.

---

## 1. Configuración del UR5 real

### 1.1 Driver

Usar `ur_robot_driver` (paquete oficial de Universal Robots para ROS 2 Jazzy):

```bash
sudo apt install ros-jazzy-ur-robot-driver ros-jazzy-ur-description
```

> **NO usar UR5e ni CB-series posteriores** sin validar — el driver tiene matices por modelo.

### 1.2 Polyscope / URCap

- [ ] Instalar URCap `external_control.urcap` en el teach pendant.
- [ ] Configurar IP del PC ROS (`Polyscope → Installation → URCaps → External Control → Host IP`).
- [ ] Crear programa con `External Control` como única instrucción.
- [ ] Probar conexión: `ros2 launch ur_robot_driver ur5.launch.py robot_ip:=192.168.1.102`.

### 1.3 Controllers YAML

Sustituir el YAML mock por el real:

```bash
ros2 launch ur5_bringup ur5_stack.launch.py \
    control_backend:=ros2_control \
    launch_ros2_control:=true \
    launch_gazebo:=false \
    controllers_yaml:=ur5_controllers_real.yaml
```

`src/ur5_description/config/ur5_controllers_real.yaml` ya está preparado con timeouts realistas (30s vs 300s sim).

### 1.4 OnRobot RG2 driver

Tres opciones según hardware:

**A. RG2 con conexión USB-RS485 a PC (recomendado):**
```bash
sudo apt install ros-jazzy-onrobot-rg2  # si está disponible
```

**B. RG2 a través de URCap (Modbus TCP por el controlador UR):**
- [ ] Configurar URCap RG2 en Polyscope.
- [ ] El gripper se controla vía script URScript desde el driver.

**C. Stub + control manual (pre-validación):**
- [ ] Mantener `gripper_attach_backend` con backend "physical" inactivo; usar el botón físico del RG2 para abrir/cerrar manualmente durante el primer pase.

---

## 2. Adaptar el stack al hardware

### 2.1 Geometría

Verificar que `geometry.yaml` representa la realidad física:

```bash
# Medir físicamente:
#   * Posición base UR5 sobre la mesa.
#   * Altura mesa.
#   * Posiciones de los objetos.
# Actualizar src/ur5_description/config/geometry.yaml en consecuencia.

pytest src/ur5_tools/test/test_geometry_yaml_consistency.py
```

### 2.2 Gazebo OFF

Lanzar el stack **sin Gazebo**:

```bash
ros2 launch ur5_bringup ur5_stack.launch.py \
    launch_gazebo:=false \
    launch_bridge:=false \
    launch_attach_backend:=false  # attach lógico no aplica en hardware
```

### 2.3 Switch de bridges

`ros_gz_bridge` no se necesita en hardware. Asegurar que `launch_bridge:=false`.

### 2.4 Frames TF

- [ ] El `world` ahora es virtual: definir un frame de referencia (ej: esquina de la mesa) y publicar `world → base_link` con un static_transform_publisher correspondiente.
- [ ] Mantener `rg2_tcp` y `rg2_pinch_center` con el mismo offset si el RG2 físico coincide con el modelo URDF. Si no, **medir y actualizar** `rg2_contact_tcp_xyz` en URDF + `geometry.yaml`.

---

## 3. Plan de validación incremental

### Paso 1 — Robot encendido sin movimiento

- [ ] Lanzar driver UR + RSP.
- [ ] `ros2 topic echo /joint_states` muestra valores reales.
- [ ] `ros2 run tf2_tools view_frames` muestra cadena completa.
- [ ] RViz2 visualiza el robot en pose actual.

### Paso 2 — Movimiento HOME en jog manual

- [ ] Activar el programa "External Control" en Polyscope.
- [ ] `ros2 service call /controller_manager/list_controllers` muestra `joint_trajectory_controller` activo.
- [ ] Mandar HOME goal vía:
  ```bash
  ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory ...
  ```
- [ ] Robot mueve a HOME despacio (velocidad MoveIt ≤ 0.05 inicialmente).

### Paso 3 — MoveIt planning

- [ ] `ros2 launch ur5_moveit_config ur5_moveit_bringup.launch.py`.
- [ ] RViz MotionPlanning panel: planificar Random pose → ejecutar (con `velocity_scaling_factor=0.1`).
- [ ] Validar que `move_group` reporta `SUCCESS` y el robot ejecuta sin oscilaciones.

### Paso 4 — Gripper RG2 manual

- [ ] `ros2 service call /gripper/open std_srvs/srv/Trigger`.
- [ ] `ros2 service call /gripper/close std_srvs/srv/Trigger`.
- [ ] Verificar visualmente que el gripper se abre/cierra correctamente.

### Paso 5 — Pick & Place sin objeto

- [ ] Ejecutar `pick_orchestrator` con `object_name=fake` (sin objeto real).
- [ ] Confirmar que las fases ejecutan: APPROACH → GRASP_DOWN → GRASP (RG2 cierra al aire) → LIFT → TRANSPORT → RELEASE.
- [ ] Sin colisiones con la mesa.

### Paso 6 — Pick & Place con objeto físico

- [ ] Colocar un objeto blando (foam, esponja) en la pose declarada.
- [ ] Ejecutar el ciclo completo.
- [ ] Verificar attach físico: el RG2 sujeta el objeto durante TRANSPORT.
- [ ] Verificar release: el objeto cae en el drop point.

### Paso 7 — Reproducibilidad

- [ ] Ejecutar 5 ciclos consecutivos sin intervención.
- [ ] Capturar KPIs (`PICK_VALIDATE_KPI_FILE`):
  - success_rate ≥ 80%.
  - duration_avg < 30s.
  - sin colisiones con la mesa.

---

## 4. Configuración crítica para HW real

| Parámetro | Sim default | HW real | Notas |
|---|---|---|---|
| `velocity_scaling_factor` | 0.30 | **0.10** | Empezar lento; subir cuando sea estable |
| `acceleration_scaling_factor` | 0.30 | **0.10** | Idem |
| `goal_time_tolerance` | 300s | **30s** | Real es más rápido |
| `path_tolerance` | 4.0 rad | **0.5 rad** | Hardware más preciso |
| `goal_tolerance` | 0.20 rad | **0.05 rad** | Hardware más preciso |
| `MAX_ATTACH_DIST` | 0.25 m | **0.05 m** | RG2 real toca; gate estricto |
| `result_timeout_sec` (PlanToPose) | 700s | **120s** | Hardware más rápido |

> ✅ Estos valores ya están parametrizados en `runtime_defaults.yaml` y `ur5_controllers_real.yaml`. Pasar el flag correcto al launch.

---

## 5. Safety pre-flight

- [ ] **Espacio libre**: 1.5 m alrededor del robot.
- [ ] **Velocity limits**: Polyscope → Safety → Speed limits ≤ 250 mm/s.
- [ ] **Force limits**: Polyscope → Safety → Force/Torque ≤ 50 N.
- [ ] **Botón E-stop accesible** en todo momento.
- [ ] **Operador presente** durante los 5 primeros ciclos.
- [ ] **Modo enseñanza apagado** durante ejecución autónoma.

---

## 6. Logging y evidencia

- [ ] Activar `evidence_logger` con `output_dir=historico/hw_<fecha>`.
- [ ] Capturar `bag` ROS 2:
  ```bash
  ros2 bag record /joint_states /tf /tf_static /pick/run_id \
      /panel_backend/result /controller_health \
      -o historico/hw_$(date +%Y%m%d_%H%M%S).bag
  ```
- [ ] Generar reporte post-sesión con `scripts/generate_latency_table.py`.

---

## 7. Rollback

Si algo va mal con el robot real:

1. **E-stop inmediato.**
2. `ros2 service call /controller_manager/switch_controller controller_manager_msgs/srv/SwitchController "{deactivate_controllers: [joint_trajectory_controller]}"`.
3. Volver a sim:
   ```bash
   ros2 launch ur5_bringup ur5_stack.launch.py launch_gazebo:=true
   ```
4. Documentar el incidente en `historico/hw_<fecha>/incident.md`.

---

## 8. Pendientes documentados (NO ejecutados)

- [ ] Calibración cámara→mesa con un objeto real (afecta percepción).
- [ ] Validación de detección con `tfm_grasping/grasp_inference` sobre imágenes reales (modelo entrenado con sim — posible domain gap).
- [ ] Test de fuerza/par límite del RG2 contra objetos reales (esponja vs cubo madera).
- [ ] CI live con UR5 real (workflow `e2e-hw-on-demand.yml` análogo al sim).

---

## 9. Recursos externos

- [Universal Robots ROS 2 driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)
- [OnRobot ROS drivers](https://github.com/Osaka-University-Harada-Laboratory/onrobot)
- [MoveIt 2 hardware tutorial](https://moveit.picknik.ai/main/doc/tutorials/quickstart_in_rviz/quickstart_in_rviz_tutorial.html)
- [Gazebo Sim → Real transfer guide](https://gazebosim.org/api/sim/8/index.html)
