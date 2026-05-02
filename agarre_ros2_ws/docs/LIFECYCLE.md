# Lifecycle Nodes — Política y estado

> Última actualización: 2026-05-01 (F11).

## 1. Política

Todo nodo crítico al pipeline pick & place **debe** ser `LifecycleNode`
(`rclpy.lifecycle.LifecycleNode`). El estado del nodo se controla
externamente vía `ros2 lifecycle set <node> <transition>` o el API
`lifecycle_msgs`.

Estados estándar:

```
unconfigured → configure → inactive → activate → ACTIVE
                                                   │
                                                   ▼
                                                deactivate → inactive
                                                   │
                                                   ▼
                                              cleanup → unconfigured
                                                   │
                                                   ▼
                                              shutdown → finalized
```

## 2. Reglas de implementación

1. **`on_configure`**: declarar parámetros, abrir servicios/actions/topics. No conectar a recursos externos pendientes.
2. **`on_activate`**: arrancar timers, suscripciones activas, comenzar a aceptar goals.
3. **`on_deactivate`**: detener timers, rechazar goals nuevos, mantener estructuras.
4. **`on_cleanup`**: liberar todo recurso obtenido en configure. El nodo vuelve a `unconfigured`.
5. **`on_shutdown`**: liberación final irrevocable.
6. Goals/peticiones cuando el nodo no está `ACTIVE` deben rechazarse con razón explícita (`node_not_active:<state>`).

## 3. Estado de los nodos críticos (2026-05-01)

| Nodo | Tipo actual | Lifecycle | Fase | Notas |
|---|---|---|---|---|
| `pick_orchestrator_lifecycle` | ✅ LifecycleNode | ✅ Pleno | F9 (cerrada) | Goals rechazados fuera de ACTIVE |
| `pick_orchestrator` (legacy F5) | Node | — (deprecated) | F11 | DeprecationWarning al arrancar |
| `world_tf_publisher` | ✅ LifecycleNode | ✅ Pleno | **F13** ✓ | Recursos segregados: subs/timer en `on_activate`, broadcasters en `on_configure` |
| `system_state_manager` | ✅ LifecycleNode | ✅ Pleno | **F13** ✓ | Recursos segregados: subs/timer en `on_activate`, publishers en `on_configure` |
| `gripper_attach_backend` | ✅ LifecycleNode | ⚠ Observable | **F13** ✓ | Callbacks expuestos; segregación de recursos en F13b si necesaria |
| `ur5_moveit_bridge` | ✅ LifecycleNode | ⚠ Observable | **F13** ✓ | Callbacks expuestos; segregación profunda en F13b (8 mixins) |
| `tf_geometry_service` | ✅ LifecycleNode | ✅ Pleno | **F16** ✓ | Servicios `/tf_geometry/world_to_base` + `/tf_geometry/compute_approach_pose` (lógica pura en `tf_geometry_logic`) |
| `release_objects_service` | Node | — | F13b | Pendiente |
| `evidence_logger` | Node | — (no necesario, solo escribe) | — | — |
| `gz_pose_bridge` | Node | LifecycleNode | F19 | Parte de optimización |
| `panel_ui_node` (objetivo) | — | LifecycleNode | F14/F15 | Cuando se extraiga del panel |

**Distinción "Pleno" vs "Observable"**:
- **Pleno**: `on_activate` crea timers/subscriptions y `on_deactivate` los destruye → permite suspensión real del nodo.
- **Observable**: el nodo expone los 5 callbacks lifecycle pero la inicialización completa permanece en `__init__` por complejidad/riesgo. Permite a `system_state_manager` coordinar y enviar señales de estado, sin re-creación granular de recursos.

**Rollback**: tag `audit-pre-f13-20260501`.

## 4. `system_state_manager` como coordinador global

Cuando todos los nodos críticos sean LifecycleNode, `system_state_manager`
arbitrará las transiciones según la fase del pipeline:

```
BOOT → controllers_up → bridge_active → orchestrator_active → READY
DEMO_RUNNING → goals aceptados solo si todos ACTIVE
SHUTDOWN → secuencia inversa (orchestrator → bridge → controllers)
```

## 5. Comandos útiles

```bash
# Ver todos los lifecycle nodes
ros2 lifecycle nodes

# Ver estado de uno
ros2 lifecycle get /pick_orchestrator_lifecycle

# Transicionar
ros2 lifecycle set /pick_orchestrator_lifecycle configure
ros2 lifecycle set /pick_orchestrator_lifecycle activate

# Listar transiciones disponibles desde el estado actual
ros2 lifecycle list /pick_orchestrator_lifecycle
```

## 6. Tests requeridos por nodo

Para cada LifecycleNode debe existir `test_<node>_lifecycle.py` que verifique:

1. `unconfigured → inactive` (configure) sin errores.
2. `inactive → ACTIVE` (activate) habilita el servicio/action.
3. Goals rechazados en `inactive` con razón explícita.
4. `ACTIVE → inactive → unconfigured` (deactivate + cleanup) libera recursos.
5. `cleanup → configure` permite re-configurar.

Se ejecutan con `--network-isolation` en CI para evitar conflictos entre tests.
