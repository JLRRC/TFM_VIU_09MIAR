# Resumen de congreso - CII 2026 UNET

## Datos para formulario

- Congreso objetivo: II Congreso Internacional de Investigacion 2026, UNET.
- Categoria recomendada: resultados de investigacion concluida.
- Modalidad: ponencia virtual asincrona en video si el resumen/articulo es aceptado.
- Area tematica recomendada: Ciencias Exactas. Alternativa secundaria: Industria y Economia, por aplicacion a automatizacion.
- Autor 1: Jesus Lozano Rodriguez.
- Autor 2: tutor/director academico, pendiente de nombre completo normalizado y afiliacion exacta.
- Advertencia: la convocatoria indica doble ciego y exige que el PDF del resumen sea anonimo. Los autores deben ir en el formulario, no en el documento PDF, salvo que la organizacion indique lo contrario.

## Titulo propuesto

HACIA UN GRASPING 2D/2.5D REPRODUCIBLE: COMPARACIÓN DE ARQUITECTURAS CNN EN ENTORNOS NO ESTRUCTURADOS

## Titulo corto

CNN PARA AGARRES 2D

## Palabras clave

deteccion de agarres; redes convolucionales; vision artificial; reproducibilidad; RGB-D

## Resumen anonimo para PDF

La deteccion visual de agarres en entornos no estructurados requiere modelos capaces de transformar informacion RGB o RGB-D en representaciones geometricas accionables, manteniendo al mismo tiempo criterios de evaluacion reproducibles. El problema abordado es la comparacion metodica de arquitecturas convolucionales para estimar rectangulos de agarre 2D/2.5D bajo un protocolo comun, con trazabilidad entre configuraciones, codigo, metricas y artefactos experimentales. El objetivo es determinar que familia de modelos ofrece mejor rendimiento relativo en la regresion de parametros de agarre cuando se controlan la modalidad de entrada, la particion de datos, las semillas de entrenamiento y las metricas de validacion. El marco conceptual se apoya en la representacion tipo Cornell, el aprendizaje profundo supervisado y la evaluacion cuantitativa mediante exito de agarre, IoU, error angular y perdida de validacion. Metodologicamente, se definieron experimentos versionados mediante archivos YAML, se uso una particion object-wise del dataset Cornell, se entrenaron tres semillas por configuracion con PyTorch y se preservaron los resultados por epoca, la mejor epoca y los resumenes agregados. Esta cadena conserva evidencia suficiente para replicar y auditar cada comparacion. Se comparo una CNN ligera configurada como `SimpleGraspCNN` e implementada como `SimpleCNN` frente a una arquitectura `ResNet18Grasp` basada en ResNet-18, considerando entradas RGB y RGB-D y el efecto de augmentation. Los resultados oficiales muestran que `ResNet18Grasp` en RGB con augmentation obtuvo el mejor rendimiento medio, con `val_success` de 0,6801 +/- 0,0245, IoU de 0,4010 +/- 0,0046 y error angular de 10,6071 +/- 1,1800 grados. La discusion indica que la arquitectura profunda con transferencia mejora de forma consistente a la CNN ligera, mientras que la incorporacion de profundidad no garantiza una mejora uniforme bajo todas las configuraciones. Como conclusion, el trabajo aporta una comparacion reproducible de arquitecturas CNN para deteccion de agarres 2D/2.5D y una base metodologica verificable para ampliar la evaluacion con metricas orientadas, nuevos datasets y estudios de generalizacion.

## Pendiente antes de enviar

- Ajustar el resumen a Times New Roman 12, interlineado 1.5, margenes 3 cm y una sola pagina/carta en PDF.
- Comprobar conteo final de palabras tras pasar a Word/LibreOffice. Objetivo: 300-500 palabras.
- Eliminar cualquier identificador de autor del PDF si se mantiene la regla de anonimato de la convocatoria.
