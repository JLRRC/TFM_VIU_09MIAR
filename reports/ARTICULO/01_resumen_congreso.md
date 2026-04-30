<!-- generado-articulo-tfm -->

# Resumen de congreso - CII 2026 UNET

## Datos para formulario

- Congreso objetivo: II Congreso Internacional de Investigacion 2026, UNET.
- Categoria recomendada: resultados de investigacion concluida.
- Modalidad: ponencia virtual asincrona en video si el resumen/articulo es aceptado.
- Area tematica recomendada: Ciencias Exactas. Alternativa secundaria: Industria y Economia, por aplicacion a automatizacion.
- Autor 1: Jesus Lozano Rodriguez.
- Autor 2: tutor del TFM, pendiente de nombre completo normalizado y afiliacion exacta.
- Advertencia: la convocatoria indica doble ciego y exige que el PDF del resumen sea anonimo. Los autores deben ir en el formulario, no en el documento PDF, salvo que la organizacion indique lo contrario.

## Titulo propuesto

COMPARACION REPRODUCIBLE DE REDES CONVOLUCIONALES PARA PREDICCION DE AGARRES ROBOTICOS EN ROS 2

## Titulo corto

CNN PARA AGARRE ROBOTICO

## Palabras clave

robotica; vision artificial; aprendizaje profundo; inteligencia artificial; automatizacion

## Resumen anonimo para PDF

La prediccion visual de agarres constituye un problema relevante en robotica de manipulacion, especialmente cuando se requiere transformar informacion sensorial en acciones ejecutables de forma trazable. La problematica abordada es la seleccion y validacion de modelos de aprendizaje profundo capaces de estimar rectangulos de agarre 2D/2.5D a partir de imagenes RGB y RGB-D, manteniendo un flujo reproducible desde el entrenamiento hasta la integracion robotica, con evaluacion automatizada y auditoria de artefactos. El objetivo del trabajo es comparar arquitecturas convolucionales para regresion de parametros de agarre y determinar una configuracion adecuada para su uso en un pipeline funcional con ROS 2, Gazebo, MoveIt 2 y un manipulador UR5 con pinza RG2. El marco conceptual combina deteccion de agarres basada en la representacion tipo Cornell, vision artificial, aprendizaje profundo e integracion robotica simulada. Metodologicamente, se definieron experimentos configurados mediante archivos YAML, con particion object-wise del dataset Cornell, tres semillas por configuracion, entrenamiento con PyTorch y evaluacion mediante `grasp success`, IoU, error angular y perdida de validacion. Se comparo una CNN ligera configurada como `SimpleGraspCNN` e implementada como `SimpleCNN`, frente a una arquitectura `ResNet18Grasp` implementada sobre ResNet-18, considerando modalidades RGB y RGB-D y el efecto de augmentation. Los resultados consolidados muestran que `ResNet18Grasp` en RGB con augmentation obtiene el mejor rendimiento oficial, con `val_success` medio de 0,6801 +/- 0,0245, IoU medio de 0,4010 +/- 0,0046 y error angular medio de 10,6071 +/- 1,1800 grados. En la discusion, estos resultados sugieren que la arquitectura profunda con transferencia ofrece una mejora clara frente a la CNN ligera, mientras que la modalidad RGB-D no garantiza una mejora uniforme en todas las configuraciones. Como conclusion, el trabajo aporta una comparacion experimental reproducible y una integracion funcional del modelo seleccionado en un entorno robotico simulado, estableciendo una base verificable para futuras extensiones con metricas orientadas, nuevos datasets y validacion fisica.

## Pendiente antes de enviar

- Ajustar el resumen a Times New Roman 12, interlineado 1.5, margenes 3 cm y una sola pagina/carta en PDF.
- Comprobar conteo final de palabras tras pasar a Word/LibreOffice. Objetivo: 300-500 palabras.
- Eliminar cualquier identificador de autor del PDF si se mantiene la regla de anonimato de la convocatoria.
