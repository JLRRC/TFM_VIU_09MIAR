# Workspace editorial del articulo

Este directorio organiza los materiales necesarios para preparar un articulo cientifico centrado en la comparacion reproducible de arquitecturas CNN para deteccion de agarres 2D/2.5D en entornos no estructurados. El contenido procede de artefactos internos ya disponibles: codigo, configuraciones, metricas, tablas, figuras y evidencias experimentales.

## Enfoque actual

- Titulo rector: **HACIA UN GRASPING 2D/2.5D REPRODUCIBLE: COMPARACIÓN DE ARQUITECTURAS CNN EN ENTORNOS NO ESTRUCTURADOS**.
- Aportacion principal: protocolo reproducible de comparacion experimental, no propuesta de una arquitectura nueva.
- Nucleo experimental del articulo: `EXP1_SIMPLE_RGB`, `EXP2_SIMPLE_RGBD`, `EXP3_RESNET18_RGB_AUGMENT` y `EXP4_RESNET18_RGBD`.
- Experimentos auxiliares: `EXP1.1_SIMPLEGRASP_RGB` y `EXP1.2_SIMPLEGRASP_RGBD`, utiles como material complementario pero no como base del argumento principal.
- Resultado central: `EXP3_RESNET18_RGB_AUGMENT` obtiene el mejor `val_success` medio entre los experimentos oficiales.
- Integracion robotica: contexto de aplicacion secundario, no eje del articulo.
- Figuras generadas: 17 PNG en `recursos/figuras/`.
- Destino inicial: II Congreso Internacional de Investigacion 2026, UNET. El primer entregable es un resumen anonimo en PDF; las normas verificadas estan en `recursos/referencias/normas_cii2026_unet.md`.

## Proximos pasos

1. Exportar `01_resumen_congreso.md` como PDF anonimo con el formato exigido por la convocatoria.
2. Confirmar datos de autores, afiliacion y declaracion CRediT para el formulario.
3. Enviar antes de la fecha limite indicada por el congreso.
4. Si el resumen es aceptado, adaptar `02_articulo_borrador.md` a la plantilla editorial asignada.
5. Seleccionar figuras centradas en metodologia y resultados; dejar latencia e integracion como material secundario si hay limite de paginas.
