# Juicio tecnico y limites

## Que si puede afirmarse con rigor

- Existe un workspace ROS 2 + Gazebo operativo y trazable para el TFM:
  - `/home/laboratorio/TFM/agarre_ros2_ws`
- La escena simulada principal esta definida y se arranca de forma reproducible:
  - mundo `ur5_mesa_objetos`
- La adquisicion de imagen simulada esta demostrada:
  - topic principal `/camera_overhead/image`
  - bridge ROS 2 <-> Gazebo operativo
- El panel carga checkpoints reales del proyecto y ejecuta inferencia sobre imagen simulada.
- La salida de la inferencia se transforma a una pose utilizable por el robot en `base_link`.
- Esa salida se publica y es consumida por un componente de MoveIt.
- Hay evidencia de ejecucion correcta del consumidor para varios casos `source=infer_model`.

## Que seria excesivo afirmar

- Que se ha demostrado de forma cerrada una manipulacion autonoma completa basada en la prediccion del modelo para el caso principal.
- Que el objeto inferido fue siempre agarrado, transportado y soltado con exito en las mismas corridas donde aparece `execute OK mode=moveit_sequence source=infer_model`.
- Que la inferencia del caso principal uso GPU de forma probada.

## Formula recomendada para el TFM

La formulacion tecnicamente segura es:

- validacion funcional de la integracion ROS 2 + Gazebo;
- escena simulada, camara, inferencia, publicacion y consumo verificados;
- con evidencia adicional de que el stack de manipulacion en simulacion tambien funciona mediante una demo manual completa.

## Formula a evitar

Evitar frases del tipo:

- "se logro una manipulacion autonoma completa basada en la red en Gazebo"
- "el sistema recoge y deposita el objeto correctamente a partir de la prediccion del modelo"

salvo que se aporte una evidencia adicional que una de forma directa la prediccion inferida con los estados finales del objeto.

## Limitaciones que conviene explicitar

- El caso mas fuerte del modelo es de nivel `B`, no `A`.
- Los logs muestran fallos intermedios antes de consolidar el flujo `publish_only` y despues `moveit_sequence`.
- La version exacta de todas las dependencias Python no esta congelada en un lockfile unico.
- La evidencia visual es suficiente para ilustrar la integracion, pero no sustituye una metrica cuantitativa de exito manipulativo en simulacion.
