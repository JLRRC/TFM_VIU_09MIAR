<!-- generado-articulo-tfm -->

# Articulo cientifico derivado del TFM

Este directorio es el espacio de trabajo para convertir el proyecto de master en un articulo cientifico. Todo el contenido generado aqui procede de fuentes internas del workspace: codigo, configuraciones, metricas, tablas, figuras y evidencias ya existentes.

## Estado actual

- Modelos inventariados: `SimpleGraspCNN`/`SimpleCNN`, `ResNet18Grasp`/`ResNetGrasp`, `SimpleGrasp` y referencias legacy no materializadas.
- Experimentos ejecutados con metricas: `EXP1_SIMPLE_RGB`, `EXP2_SIMPLE_RGBD`, `EXP3_RESNET18_RGB_AUGMENT`, `EXP4_RESNET18_RGBD`, `EXP1.1_SIMPLEGRASP_RGB`, `EXP1.2_SIMPLEGRASP_RGBD`.
- Configuraciones adicionales localizadas: `EXP_METHOD_V2_RGB` y `EXP_TEMPLATE`.
- Modelo final documentado para inferencia reproducible: `EXP3_RESNET18_RGB_AUGMENT` con `ResNet18Grasp`, seed 0 en el preset de memoria.
- Figuras generadas para el articulo: 17 PNG en `recursos/figuras/`.
- Destino inicial actualizado: II Congreso Internacional de Investigacion 2026, UNET. El primer entregable es un resumen anonimo en PDF; las normas verificadas estan en `recursos/referencias/normas_cii2026_unet.md`.

## Proximos pasos

1. Cerrar y exportar el resumen de `01_resumen_congreso.md` en formato PDF anonimo.
2. Confirmar datos del tutor, afiliacion y CRediT para el formulario.
3. Enviar antes de la fecha limite indicada por la web del congreso.
4. Si se acepta el resumen, convertir `02_articulo_borrador.md` a la plantilla editorial asignada.
5. Seleccionar 4-6 figuras finales desde `05_figuras_y_tablas.md`.
