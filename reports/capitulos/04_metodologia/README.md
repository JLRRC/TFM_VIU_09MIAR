# Capitulo 04. Metodologia

- Paginas del PDF: 33-54
- Ruta: reports/capitulos/04_metodologia

## Contenido

### Ilustraciones

- Ilustracion 4-1: Vision global del pipeline experimental
- Ilustracion 4-2: Ejemplo de anotaciones Cornell sobre una imagen
- Ilustracion 4-3: Flujo del pipeline: auditoría → índices limpios → dataset Subset estricto
- Ilustracion 4-4: Ejemplo de evaluación tipo Cornell con fallo por incumplimiento de umbrales de IoU y/o Δθ
- Ilustracion 4-5: Galería cualitativa: 4 aciertos + 4 fallos con explicación del tipo de fallo
- Ilustracion 4-6: Arquitectura ROS 2 del entorno simulado (nodos y tópicos)
- Ilustracion 4-7: Entorno de emulación ROS 2/Gazebo: ejemplo de consistencia visual (overlay) y consumo de la hipótesis de agarre en la escena table-top
### Tablas

- Tabla 4-1: Fases del trabajo, entradas, salidas y evidencias de verificación
- Tabla 4-2: Estadísticas del split utilizado en el conjunto experimental final
- Tabla 4-3: Transformaciones de data augmentation y cómo se actualizan las etiquetas Cx, Cy, w, h, θ
- Tabla 4-4: Comparativa estructural de los modelos (A vs B)
- Tabla 4-5: Configuración experimental por experimento
- Tabla 4-6: Hiperparámetros de entrenamiento por experimento (según YAML asociado)
- Tabla 4-7: Definición de métricas de evaluación (Cornell): IoU, Δθ y grasp success
- Tabla 4-8: Protocolo de medición de latencia
- Tabla 4-9: Especificaciones de hardware utilizadas
### Artefactos

- Artefacto 4-A1: Diagrama de arquitectura del modelo ResNet18Grasp
- Artefacto 4-A2: Diagrama de arquitectura del modelo SimpleGraspCNN
