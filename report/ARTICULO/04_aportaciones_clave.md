<!-- generado-articulo-tfm -->

# Aportaciones clave

## Aportacion real

- Comparacion reproducible de dos familias oficiales de modelos (`SimpleGraspCNN`/`SimpleCNN` y `ResNet18Grasp`/`ResNetGrasp`) en RGB y RGB-D.
- Evidencia multisemilla con metricas por epoca, mejor epoca, latencia y resumen agregado.
- Integracion funcional de inferencia en ROS 2/Gazebo/MoveIt 2 con seleccion de checkpoint final.
- Trazabilidad explicita entre memoria, codigo, configuraciones y resultados.

## Novedad defendible

La novedad defendible no es una arquitectura nueva, sino la combinacion de comparacion reproducible, curacion de artefactos, analisis de trade-offs rendimiento/latencia y despliegue funcional en un pipeline robotico simulado.

## Que no debe prometer el articulo

- No prometer rendimiento superior al estado del arte.
- No afirmar validacion en robot fisico si no se aporta evidencia directa.
- No mezclar metricas oficiales historicas con la variante metodologica posterior.
- No presentar `SimpleGraspCNN` como clase Python real; es nombre de configuracion que instancia `SimpleCNN`.

## Mensaje principal recomendado

`ResNet18Grasp` RGB con augmentation ofrece el mejor equilibrio de rendimiento dentro del bloque oficial, mientras que el workspace aporta trazabilidad suficiente para reproducir y auditar la comparacion completa.
