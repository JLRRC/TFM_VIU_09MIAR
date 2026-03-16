# Log de trabajo - Comentarios 2, 3 y 4 del tutor

## Objetivo

Preparar material visual y textual trazable para responder a los comentarios 2, 3 y 4 sobre la seccion 5.2.1 del TFM, sin alterar el contenido experimental ni desordenar el workspace.

## Restricciones operativas

- No tocar ni usar rutas dentro de carpetas `BORRAR`.
- Mantener separacion entre fuentes, intermedios, scripts, textos y finales.
- Integrarse en la estructura real de ilustraciones del TFM.
- No sobrescribir silenciosamente archivos relevantes.

## Estructura de trabajo creada

- Carpeta raiz de revision: `reports/tfm_visual_revision/`
- Subcarpetas:
  - `logs/`
  - `inventory/`
  - `intermediate/`
  - `scripts/`
  - `selections/`
  - `captions/`
  - `final/`

## Rutas revisadas

- `docs/`
- `reports/figures/`
- `reports/tfm_figuras_cap5_1/`
- `experiments/`
- `config/`
- `data/processed/cornell/splits/object_wise/`
- `src/evaluation/`
- `src/training/`
- `scripts/`

## Hallazgos iniciales

- Documento maestro localizado:
  - `docs/TFM_V7.9.pdf`
- Carpeta maestra coherente para ilustraciones del TFM:
  - `reports/figures/`
- Carpeta previa de trabajo especifica del capitulo 5:
  - `reports/tfm_figuras_cap5_1/`
- Figuras cualitativas candidatas localizadas:
  - `reports/figures/visualizaciones_cualitativas_EXP1_SIMPLE_RGB.png`
  - `reports/figures/visualizaciones_cualitativas_EXP3_RESNET18_RGB_AUGMENT.png`
- La figura a revisar en el PDF es la `Ilustracion 5-14`, ubicada en la seccion 5.2.1, donde mezcla aciertos y fallos.
- La taxonomia E1-E4 ya existe en el PDF:
  - E1: error angular
  - E2: bajo solapamiento
  - E3: tamano/apertura no plausible
  - E4: confusion por fondo u oclusion

## Evidencia textual localizada en el PDF

- `Ilustracion 5-14. Galeria cualitativa en validacion con superposicion GT (verde discontinuo) vs Pred (rojo continuo): fila superior (ejemplos con mayor IoU, incluyendo aciertos) y fila inferior (fallos por bajo solape y errores tipificados E1-E4). Se muestran IoU y Delta theta por ejemplo.`
- La seccion `5.2.1 Tipos de acierto observados` remite a esa misma ilustracion y por eso debe reescribirse si la figura se separa.

## Scripts y fuentes identificados

- Script de overlays existente:
  - `scripts/generate_qualitative_overlays.py`
- Script de evaluacion existente:
  - `scripts/evaluate.py`
- Configuraciones relevantes:
  - `config/exp1_simple_rgb.yaml`
  - `config/exp3_resnet18_rgb_augment.yaml`
- Checkpoints disponibles:
  - `experiments/EXP1_SIMPLE_RGB/seed_{0,1,2}/checkpoints/best.pth`
  - `experiments/EXP3_RESNET18_RGB_AUGMENT/seed_{0,1,2}/checkpoints/best.pth`
- Split de validacion disponible:
  - `data/processed/cornell/splits/object_wise/val.csv`

## Ambiguedades detectadas

- No se ha localizado un archivo fuente explicito de la galeria final usada como `Ilustracion 5-14`.
- No se ha localizado un `predictions.csv` persistido para las figuras cualitativas ya exportadas.
- `scripts/evaluate.py` exporta `predictions.csv`, pero no pasa `csv_path` al `Evaluator`; por tanto, ese flujo no garantiza por si solo el uso de multi-GT corregido en la exportacion de casos.

## Decisiones adoptadas

- Usar `reports/figures/` como ubicacion final coherente para las nuevas ilustraciones del TFM.
- Mantener copia trazable de los finales en `reports/tfm_visual_revision/final/`.
- Construir una utilidad aislada en `reports/tfm_visual_revision/scripts/` para reconstruir casos cualitativos con criterio reproducible y soporte multi-GT, sin modificar los scripts experimentales nucleares.
- Documentar todas las selecciones y clasificaciones manuales con criterio explicito.

## Registro de acciones

- 2026-03-15: creada la estructura `reports/tfm_visual_revision/`.
- 2026-03-15: localizado `docs/TFM_V7.9.pdf` como version actual del TFM.
- 2026-03-15: identificado `reports/figures/` como carpeta maestra de ilustraciones.
- 2026-03-15: identificada `Ilustracion 5-14` del PDF como figura actual que mezcla aciertos y fallos.
- 2026-03-15: extraida la pagina 66 del PDF en `intermediate/tfm_v7_9_page66-66.png` para verificar visualmente la figura actual y su contexto textual.
- 2026-03-15: detectado que `scripts/evaluate.py` no pasa `csv_path` al `Evaluator`, por lo que no se ha usado como base directa para esta revision.
- 2026-03-15: creado `scripts/rebuild_qualitative_inventory.py` para reconstruir un inventario reproducible por imagen unica y con soporte multi-GT.
- 2026-03-15: ejecutado `venv/bin/python reports/tfm_visual_revision/scripts/rebuild_qualitative_inventory.py`.
- 2026-03-15: generados:
  - `inventory/qualitative_inventory_EXP3_RESNET18_RGB_AUGMENT_seed_0.csv`
  - `inventory/qualitative_inventory_EXP1_SIMPLE_RGB_seed_0.csv`
  - `inventory/qualitative_inventory_combined.csv`
  - `inventory/qualitative_inventory_summary.csv`
  - overlays individuales en `intermediate/overlays/`
- 2026-03-15: generadas hojas de contacto auxiliares en `intermediate/contact_sheets/` para revisar candidatos de aciertos y de E1-E4.
- 2026-03-15: fijada la seleccion definitiva en `selections/seleccion_casos.md`.
- 2026-03-15: creada la taxonomia operativa documentada en `inventory/taxonomia_errores_E1_E4.md`.
- 2026-03-15: creado `scripts/compose_final_revision_figures.py` para componer las figuras finales a partir del inventario.
- 2026-03-15: ejecutado `venv/bin/python reports/tfm_visual_revision/scripts/compose_final_revision_figures.py`.
- 2026-03-15: generadas figuras finales:
  - `final/cap5_aciertos_cualitativos_v1.png`
  - `final/cap5_fallos_cualitativos_E1_E4_v1.png`
  - `final/cap5_caso_ilustrativo_v1.png`
  - y sus equivalentes PDF
- 2026-03-15: copiadas las versiones finales listas para insercion en `reports/figures/` con el mismo nombre de archivo.
- 2026-03-15: redactadas captions y texto asociado en `captions/`.

## Decision sobre sustitucion de figuras

- No se ha sobrescrito ningun archivo previo de `reports/figures/`.
- La nueva pareja de figuras `cap5_aciertos_cualitativos_v1.*` y `cap5_fallos_cualitativos_E1_E4_v1.*` debe entenderse como sustituta conceptual de la antigua `Ilustracion 5-14` del PDF, que mezclaba ambos tipos de casos en una sola galeria.
- La figura `cap5_caso_ilustrativo_v1.*` se propone como apoyo especifico para el comentario 4.
