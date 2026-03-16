# Nivel de evidencia

## Escala usada

- Nivel A:
  - manipulacion completa demostrada con evidencia suficiente para afirmar un pick funcional de extremo a extremo.
- Nivel B:
  - pipeline funcional `percepcion -> publicacion -> consumo` verificado, con ejecucion coherente del consumidor, pero sin prueba equivalente de agarre fisico completo asociado a esa misma prediccion.
- Nivel C:
  - integracion parcial o indicios, insuficientes para una afirmacion fuerte de funcionamiento.

## Clasificacion de los casos revisados

### Caso 1. Pipeline con modelo e integracion a MoveIt

- Nivel asignado:
  - `B`
- Razon:
  - hay escena simulada;
  - hay adquisicion de imagen;
  - hay carga de modelo;
  - hay inferencia y registro de `grasp_base`;
  - hay publicacion ROS 2;
  - hay consumidor MoveIt operativo;
  - hay varios `execute OK mode=moveit_sequence source=infer_model`.
- Lo que falta para subir a `A`:
  - prueba vinculada a esa misma prediccion de que el objeto fue realmente agarrado, transportado y liberado con exito.

### Caso 2. Demo manual de manipulacion completa

- Nivel asignado:
  - `A` para el stack de manipulacion simulado en si mismo
- Razon:
  - el log documenta estados `GRASPED`, `CARRIED`, `RELEASED` y `Secuencia PICK completada`.
- Limite:
  - no debe presentarse como evidencia de percepcion basada en el modelo.

### Caso 3. Publicacion sin movimiento

- Nivel asignado:
  - `C`
- Razon:
  - demuestra una etapa intermedia de integracion, pero no un pipeline completo.

## Nivel de evidencia recomendado para la subseccion del TFM

- Nivel global recomendado:
  - `B`

## Juicio operativo

La subseccion de resultados ROS 2 + Gazebo puede afirmar con rigor una validacion funcional de integracion entre simulacion, percepcion y consumo por MoveIt. No debe afirmar, con el material revisado, una manipulacion autonoma completa guiada por la prediccion del modelo.
