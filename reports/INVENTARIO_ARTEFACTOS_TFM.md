# Inventario de artefactos del TFM

Inventario alineado con la numeracion del documento `TFM_Jesus_Lozano_V10.pdf`.
La tabla maestra de correspondencia adicional queda en `reports/CORRESPONDENCIA_PDF_ARTEFACTOS_TFM.md` y `reports/CORRESPONDENCIA_PDF_ARTEFACTOS_TFM.csv`.

## Capitulo 01. Introduccion

- Paginas: 11-13

### Ilustraciones

- Ilustracion 1-1: Ejemplos de escenarios no estructurados para deteccion de poses de agarre (p. 11)
- Ilustracion 1-2: Representacion 2D/2.5D del agarre como rectangulo orientado (p. 12)
- Ilustracion 1-3: Diagrama general del pipeline del trabajo e integracion (p. 13)

### Tablas

- Sin elementos catalogados.

### Artefactos

- Sin elementos catalogados.

## Capitulo 02. Objetivos y alcance

- Paginas: 14-17

### Ilustraciones

- Ilustracion 2-1: Mapa de trazabilidad del pipeline (dataset -> modelo -> metricas -> ROS 2/Gazebo) (p. 15)

### Tablas

- Tabla 2-1: Trazabilidad entre objetivos especificos y evidencias en el documento (p. 15)
- Tabla 2-2: Fuera de alcance vs extensiones futuras (p. 17)

### Artefactos

- Sin elementos catalogados.

## Capitulo 03. Estado del Arte y Marco teorico

- Paginas: 18-32

### Ilustraciones

- Ilustracion 3-1: Taxonomia del problema de agarre: 2D/2.5D vs 6-DoF (p. 19)
- Ilustracion 3-2: Ejemplo conceptual de mapas densos (calidad, angulo, apertura) estilo GG-CNN (p. 20)
- Ilustracion 3-3: Ejemplos de degradacion por oclusion/fondo: tipologia de fallos (p. 22)
- Ilustracion 3-4: Mapa de posicionamiento: estado del arte -> decisiones del trabajo (p. 24)
- Ilustracion 3-5: Parametros del rectangulo de agarre (Cx, Cy, w, h, theta) sobre una imagen (p. 28)
- Ilustracion 3-6: Interpretacion geometrica de IoU y delta theta en rectangulos orientados (p. 32)

### Tablas

- Tabla 3-1: Comparativa sintetica de enfoques 2D/2.5D: mapas densos vs regresion parametrica (p. 21)
- Tabla 3-2: Datasets relevantes: Cornell/Jacquard vs GraspNet/ACRONYM (p. 25)

### Artefactos

- Sin elementos catalogados.

## Capitulo 04. Metodologia

- Paginas: 33-54

### Ilustraciones

- Ilustracion 4-1: Vision global del pipeline experimental (p. 33)
- Ilustracion 4-2: Ejemplo de anotaciones Cornell sobre una imagen (p. 36)
- Ilustracion 4-3: Flujo del pipeline: auditoria -> indices limpios -> dataset Subset estricto (p. 38)
- Ilustracion 4-4: Ejemplo de evaluacion tipo Cornell con fallo por incumplimiento de umbrales de IoU y/o delta theta (p. 47)
- Ilustracion 4-5: Galeria cualitativa: 4 aciertos + 4 fallos con explicacion del tipo de fallo (p. 48)
- Ilustracion 4-6: Arquitectura ROS 2 del entorno simulado (nodos y topicos) (p. 50)
- Ilustracion 4-7: Entorno de emulacion ROS 2/Gazebo: ejemplo de consistencia visual y consumo de la hipotesis de agarre (p. 52)

### Tablas

- Tabla 4-1: Fases del trabajo, entradas, salidas y evidencias de verificacion (p. 35)
- Tabla 4-2: Estadisticas del split utilizado en el conjunto experimental final (p. 37)
- Tabla 4-3: Transformaciones de data augmentation y como se actualizan las etiquetas (p. 39)
- Tabla 4-4: Comparativa estructural de los modelos (A vs B) (p. 44)
- Tabla 4-5: Configuracion experimental por experimento (p. 45)
- Tabla 4-6: Hiperparametros de entrenamiento por experimento (p. 46)
- Tabla 4-7: Definicion de metricas de evaluacion (Cornell): IoU, delta theta y grasp success (p. 47)
- Tabla 4-8: Protocolo de medicion de latencia (p. 49)
- Tabla 4-9: Especificaciones de hardware utilizadas (p. 53)

### Artefactos

- Artefacto 4-A1: Diagrama de arquitectura del modelo ResNet18Grasp (p. 41)
- Artefacto 4-A2: Diagrama de arquitectura del modelo SimpleGraspCNN (p. 42)

## Capitulo 05. Resultados y discusion

- Paginas: 56-77

### Ilustraciones

- Ilustracion 5-1: Curvas de perdida (train_loss y val_loss) para EXP1 (p. 57)
- Ilustracion 5-2: Curvas de perdida (train_loss y val_loss) para EXP2 (p. 57)
- Ilustracion 5-3: Curvas de perdida (train_loss y val_loss) para EXP3 (p. 57)
- Ilustracion 5-4: Curvas de perdida (train_loss y val_loss) para EXP4 (p. 57)
- Ilustracion 5-5: Comparativa de val_success por seed y experimento en best_epoch (p. 58)
- Ilustracion 5-6: Comparativa de val_loss por seed y experimento en best_epoch (p. 59)
- Ilustracion 5-7: Evolucion del exito de agarre en validacion por epoca (p. 59)
- Ilustracion 5-8: Evolucion del IoU medio en validacion por epoca (p. 60)
- Ilustracion 5-9: Evolucion del error angular medio en validacion por epoca (p. 60)
- Ilustracion 5-10: Exito final de agarre en validacion agregado por experimento (p. 62)
- Ilustracion 5-11: IoU medio final en validacion agregado por experimento (p. 62)
- Ilustracion 5-12: Error angular medio final en validacion agregado por experimento (p. 62)
- Ilustracion 5-13: Aciertos representativos en validacion para EXP3_RESNET18_RGB_AUGMENT (p. 64)
- Ilustracion 5-14: Fallos cualitativos representativos en validacion organizados por tipologia (p. 66)
- Ilustracion 5-15: Caso ilustrativo de objeto pequeno en la escena (p. 68)
- Ilustracion 5-16: Evidencia funcional del pipeline percepcion-publicacion-consumo en ROS 2 (p. 71)
- Ilustracion 5-17: Resultado de inferencia del modelo EXP3_RESNET18_RGB_AUGMENT sobre la imagen simulada (p. 72)
- Ilustracion 5-18: Evidencia funcional adicional del pipeline percepcion-publicacion-consumo en ROS 2 (p. 73)

### Tablas

- Tabla 5-1: Resultados agregados en validacion (best_epoch por ejecucion) bajo split object-wise (p. 61)
- Tabla 5-2: Resumen de metricas finales por experimento en validacion (p. 62)
- Tabla 5-3: Medicion de latencia de inferencia por experimento y dispositivo (p. 69)
- Tabla 5-4: Comparativa por modalidad entre SimpleGraspCNN y ResNet18Grasp (p. 77)

### Artefactos

- Artefacto 5-A1: Resumen tecnico para la redaccion de la subseccion ROS 2 + Gazebo
- Artefacto 5-A2: Artefacto principal de inferencia y grasp publicado
- Artefacto 5-A3: Resumen tabular completo del bloque experimental

## Capitulo 06. Conclusiones y trabajos futuros

- Paginas: 78-83

### Ilustraciones

- Sin elementos catalogados.

### Tablas

- Tabla 6-1: Sintesis de cumplimiento de objetivos (p. 79)

### Artefactos

- Sin elementos catalogados.

## Capitulo 07. Referencias bibliograficas

- Paginas: 84-85

### Ilustraciones

- Sin elementos catalogados.

### Tablas

- Sin elementos catalogados.

### Artefactos

- Sin elementos catalogados.

## Capitulo 08. Anexos

- Paginas: 86-88

### Ilustraciones

- Sin elementos catalogados.

### Tablas

- Tabla 8-1: Resultados por semilla y experimento en la mejor epoca de validacion (p. 86)
- Tabla 8-2: Resultados de validacion del experimento de referencia para la integracion ROS 2 (EXP4_RESNET18_RGBD) (p. 87)
- Tabla 8-3: Resumen de experimentos base en validacion (p. 87)
- Tabla 8-4: Comparativa por modalidad entre SimpleGraspCNN y ResNet18Grasp (p. 88)

### Artefactos

- Artefacto 8-A1: Documento PDF del TFM utilizado como referencia editorial
