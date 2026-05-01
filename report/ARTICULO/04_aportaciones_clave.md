# Aportaciones clave

## Aportacion principal

- Comparacion reproducible de dos familias oficiales de modelos (`SimpleGraspCNN`/`SimpleCNN` y `ResNet18Grasp`/`ResNetGrasp`) para deteccion de agarres 2D/2.5D.
- Protocolo comun con particion object-wise, configuraciones YAML, tres semillas por experimento, metricas por epoca y agregacion por mejor epoca.
- Analisis del efecto combinado de arquitectura, modalidad de entrada RGB/RGB-D y augmentation.
- Trazabilidad explicita entre configuraciones, codigo, metricas, tablas y figuras.

## Novedad defendible

La novedad defendible no es una arquitectura nueva, sino la curacion metodologica de un benchmark reproducible y auditable. El valor del articulo esta en mostrar como cambian los resultados cuando se controla la arquitectura, la modalidad de entrada y el protocolo de evaluacion, y en separar con claridad resultados oficiales, extensiones auxiliares y trabajo pendiente.

## Que no debe prometer el articulo

- No prometer rendimiento superior al estado del arte.
- No presentar validacion fisica si no se aporta evidencia directa.
- No convertir la integracion robotica en contribucion principal.
- No mezclar metricas oficiales historicas con la variante metodologica posterior.
- No presentar `SimpleGraspCNN` como clase Python real; es nombre de configuracion que instancia `SimpleCNN`.

## Mensaje principal recomendado

`ResNet18Grasp` RGB con augmentation ofrece el mejor `val_success` dentro del bloque oficial, y el articulo aporta una metodologia reproducible para auditar esa comparacion sin sobredimensionar sus conclusiones.
