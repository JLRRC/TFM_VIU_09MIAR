# F8 — Migración lógica TFM al backend (plan de cierre)

## Estado actual

El paquete `tfm_grasping` ya tiene los componentes pure de la
inferencia (model, perception, geometry, grasp_selector). El panel Qt
mantiene 1860 LOC repartidos entre `panel_tfm_inference.py` (829) y
`panel_tfm_science.py` (1031) que mezclan UI con orquestación de
inferencia.

## Hecho en F8 (2026-05-10)

* Extraído `tfm_grasping/grasp_geometry_helpers.py` con los helpers
  puros de `panel_tfm_geometry.py` (clamp, normalize_angle,
  compute_minor_axis_from_grasp_rect, compute_rg2_preopen_from_minor_width).
* 15 tests offline (clamp, normalize_angle parametrizado, minor_axis
  cases, RG2 pre-open).
* Añadida interfaz **`InferGrasp.srv`** en `ur5_panel_interfaces` para
  que el panel pueda delegar la inferencia al backend.
* Documentación de plan F8b para completar la migración.

## Plan F8b (cuando haya tiempo + live test)

### Paso 1 — Nodo backend `tfm_inference_node`

Crear `src/tfm_grasping/tfm_grasping/tfm_inference_node.py`:

* LifecycleNode que carga el modelo en `on_configure`.
* Suscribe `/camera_overhead/image` y `/camera_overhead/depth_image`
  como buffer rolling (último frame).
* Service `/tfm/infer_grasp` (`InferGrasp.srv`) que ejecuta la
  inferencia sobre el último frame depth + RGB y devuelve el grasp
  estructurado.
* Publica `/tfm/grasp_prediction` (PoseStamped + ext) para overlay UI.

### Paso 2 — Panel deja de hacer inferencia in-process

* Reemplazar `tfm_infer()` en `panel_tfm_inference.py` por una
  llamada al service `/tfm/infer_grasp`.
* `panel_tfm_science.py` se reduce a UI/visualización; la geometría
  ya está en el helper extraído.
* LOC esperado tras migración:
  * panel_tfm_inference.py: 829 → ~200 (cliente del service + UI)
  * panel_tfm_science.py: 1031 → ~500 (overlay + UI; no inference)

### Paso 3 — Tests integración

* `test_tfm_inference_node_smoke.py` con `launch_testing` lanzando
  el nodo + un cliente que envía `InferGrasp.Request`.
* Validar que el panel funciona en modo "stack inference" (consumiendo
  el service) además del legacy in-process.

## Riesgos

* **Modelo en memoria**: cargar el modelo en el backend en lugar del
  panel duplicaría memoria si el panel mantiene el modelo legacy
  como fallback. Migración completa requiere borrar la carga en panel.
* **Latencia**: añade 1 RTT ROS al pipeline de inferencia. Medir antes
  de cerrar — si > 50 ms en local, considerar topic-based en lugar de
  service.
* **TFM_INFER_DEVICE**: env var leída en `tfm_grasping/model.py:57`
  hoy. Tras F8b, hay que asegurar que el backend lee la variable y el
  panel ya no la consume.

## Tag rollback

```bash
git tag pre-f8b-tfm-backend-migration
```
