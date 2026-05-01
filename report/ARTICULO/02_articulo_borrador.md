# Titulo

COMPARACIÓN REPRODUCIBLE DE ARQUITECTURAS CNN PARA DETECCIÓN DE AGARRES 2D/2.5D EN ENTORNOS NO ESTRUCTURADOS

## Resumen

La deteccion de agarres a partir de imagenes RGB y RGB-D es un problema central en manipulacion robotica cuando las escenas contienen variabilidad de objetos, iluminacion y geometria. Este articulo presenta una comparacion reproducible de arquitecturas convolucionales para regresion de rectangulos de agarre 2D/2.5D bajo una representacion tipo Cornell. El estudio contrasta una CNN ligera, configurada como `SimpleGraspCNN` e implementada como `SimpleCNN`, con una arquitectura `ResNet18Grasp` basada en ResNet-18. La metodologia fija particion object-wise, configuraciones YAML versionadas, tres semillas por experimento, entrenamiento con PyTorch y evaluacion mediante `grasp success`, IoU, error angular y perdida de validacion. Los resultados oficiales muestran que `ResNet18Grasp` con entrada RGB y augmentation (`EXP3_RESNET18_RGB_AUGMENT`) obtiene el mejor rendimiento medio entre los experimentos consolidados: `val_success` 0,6801 +/- 0,0245, IoU 0,4010 +/- 0,0046 y error angular 10,6071 +/- 1,1800 grados. La comparacion evidencia que la arquitectura profunda con transferencia supera a la CNN ligera en las modalidades evaluadas, mientras que la entrada RGB-D no produce una mejora uniforme frente a RGB con augmentation. La contribucion principal no reside en proponer una nueva arquitectura, sino en ofrecer un protocolo auditable de comparacion, curacion de artefactos y analisis de resultados para deteccion de agarres 2D/2.5D en condiciones no estructuradas.

## Palabras clave

deteccion de agarres; redes convolucionales; ResNet-18; RGB-D; Cornell grasping dataset; reproducibilidad

## Introduccion

La prediccion visual de agarres consiste en estimar una configuracion geometrica que permita aproximar una accion de prension a partir de informacion sensorial. En entornos no estructurados, esta tarea resulta especialmente exigente por la variacion de forma, posicion, escala y textura de los objetos. Una formulacion habitual representa cada agarre como un rectangulo orientado definido por centro, anchura, altura y angulo. Esta representacion reduce la complejidad frente a formulaciones 6-DoF completas y permite comparar modelos mediante protocolos cuantitativos derivados del Cornell Grasping Dataset.

Aunque existen arquitecturas especializadas para grasping visual, muchos sistemas aplicados siguen dependiendo de decisiones experimentales que no siempre se documentan con suficiente detalle: modalidad RGB o RGB-D, estrategia de augmentation, seleccion de arquitectura, particion de datos, semillas, criterio de perdida y definicion exacta de las metricas. Estas decisiones pueden modificar la lectura de los resultados tanto como la arquitectura seleccionada. Por ello, el foco de este trabajo es metodologico: establecer una comparacion trazable de arquitecturas CNN para deteccion de agarres 2D/2.5D y delimitar que conclusiones son defendibles a partir de los artefactos experimentales disponibles.

El estudio compara una CNN ligera frente a una variante profunda basada en ResNet-18, manteniendo un protocolo comun de entrenamiento y evaluacion. La integracion robotica se considera un contexto de aplicacion secundario; el nucleo del articulo es la reproducibilidad del proceso experimental, la curacion de resultados y la interpretacion prudente de los trade-offs entre arquitectura, modalidad de entrada y augmentation.

## Estado del arte

La literatura de grasping visual ha utilizado ampliamente representaciones rectangulares de agarre por su relacion directa con pinzas paralelas y por facilitar la evaluacion automatizada sobre datasets anotados. Cornell establecio un marco temprano para deteccion de rectangulos de agarre, mientras que trabajos posteriores ampliaron el problema hacia mapas densos de calidad, angulo y apertura, o hacia datasets de mayor escala como Jacquard y GraspNet.

En este contexto, las CNN ligeras ofrecen bajo coste computacional y facilidad de despliegue, pero pueden limitar la extraccion de caracteristicas visuales complejas. Las arquitecturas profundas con transferencia, como ResNet-18, incorporan una representacion visual mas rica y suelen mejorar la generalizacion en tareas de vision, aunque con mayor numero de parametros. La comparacion RGB frente a RGB-D tambien permanece abierta: la profundidad puede aportar informacion geometrica, pero su utilidad depende de la calidad del preprocesamiento, de la arquitectura y del protocolo de entrenamiento.

Este articulo se situa en esa linea de evaluacion comparativa. No busca reclamar superioridad frente al estado del arte, sino aportar una lectura reproducible y acotada de varias configuraciones CNN bajo un mismo protocolo experimental.

## Metodologia

El protocolo se organiza como una cadena reproducible de configuracion, entrenamiento, evaluacion y agregacion de resultados. Cada experimento se define mediante un archivo YAML que fija arquitectura, modalidad de entrada, augmentation, optimizador, numero de epocas, batch size, criterio de perdida y rutas de datos. El dataset Cornell procesado se divide con particion object-wise para evitar que instancias del mismo objeto contaminen simultaneamente entrenamiento y validacion.

La entrada del modelo se normaliza a imagenes de 224 x 224 pixeles. Las configuraciones RGB usan tres canales, mientras que las RGB-D incorporan un cuarto canal de profundidad. Los experimentos con augmentation aplican transformaciones de entrenamiento controladas, incluyendo rotaciones, volteo horizontal y variaciones de color cuando procede. La salida de los modelos es un vector de cinco parametros: `cx`, `cy`, `w`, `h` y `angle`.

La evaluacion oficial usa entrenamiento con `SmoothL1Loss` y reporta `val_success`, `val_iou`, `val_angle_deg` y `val_loss` en la mejor epoca de validacion. Para cada configuracion se ejecutan tres semillas, y los resultados se agregan como media y desviacion entre semillas. Existe una variante metodologica posterior con perdida angular periodica e IoU orientada, pero se mantiene separada para no mezclar resultados oficiales con extensiones no consolidadas.

La trazabilidad se apoya en tres niveles: configuraciones versionadas, metricas por epoca y resumenes de mejor epoca por semilla. Esta organizacion permite auditar la correspondencia entre arquitectura declarada, experimento ejecutado y cifras incluidas en el articulo.

## Modelos evaluados

| modelo en configuracion | clase implementada | modalidad | descripcion | papel en el estudio |
| --- | --- | --- | --- | --- |
| `SimpleGraspCNN` | `SimpleCNN` | RGB / RGB-D | CNN ligera con bloques convolucionales, normalizacion, pooling adaptativo y MLP final de cinco salidas. | Baseline oficial de baja complejidad. |
| `ResNet18Grasp` | `ResNetGrasp` | RGB / RGB-D | ResNet-18 con capa final adaptada a cinco parametros de agarre; la primera convolucion se ajusta cuando la entrada tiene cuatro canales. | Arquitectura profunda principal para comparar transferencia y capacidad representacional. |
| `SimpleGrasp` | `SimpleGrasp` | RGB / RGB-D | Variante ligera posterior alineada con el diseno teorico documentado, entrenada en experimentos auxiliares. | Evidencia complementaria; no sustituye el bloque oficial. |

La distincion entre nombre de configuracion y clase implementada es relevante para evitar ambiguedades: `SimpleGraspCNN` instancia realmente `SimpleCNN`, y `ResNet18Grasp` instancia `ResNetGrasp`.

## Protocolo experimental

El nucleo del articulo esta formado por cuatro experimentos oficiales. Los experimentos auxiliares `EXP1.1` y `EXP1.2` documentan una variante posterior, pero se reservan para material complementario.

| experimento | modelo_config | clase_real | modalidad | canales | augmentation | epocas | criterio | estado |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `EXP1_SIMPLE_RGB` | `SimpleGraspCNN` | `SimpleCNN` | RGB | 3 | no | 10 | `smooth_l1` | oficial |
| `EXP2_SIMPLE_RGBD` | `SimpleGraspCNN` | `SimpleCNN` | RGB-D | 4 | si | 10 | `smooth_l1` | oficial |
| `EXP3_RESNET18_RGB_AUGMENT` | `ResNet18Grasp` | `ResNetGrasp` | RGB | 3 | si | 50 | `smooth_l1` | principal |
| `EXP4_RESNET18_RGBD` | `ResNet18Grasp` | `ResNetGrasp` | RGB-D | 4 | no | 50 | `smooth_l1` | oficial |

## Resultados

| experimento | modelo | modalidad | success | IoU | error angular | val_loss |
| --- | --- | --- | --- | --- | --- | --- |
| `EXP1_SIMPLE_RGB` | `SimpleGraspCNN` | RGB | 0,2741 +/- 0,0440 | 0,2254 +/- 0,0496 | 17,1910 +/- 1,3745 | 0,0321 +/- 0,0008 |
| `EXP2_SIMPLE_RGBD` | `SimpleGraspCNN` | RGB-D | 0,3805 +/- 0,0051 | 0,3169 +/- 0,0033 | 17,8730 +/- 0,3608 | 0,0320 +/- 0,0001 |
| `EXP3_RESNET18_RGB_AUGMENT` | `ResNet18Grasp` | RGB | 0,6801 +/- 0,0245 | 0,4010 +/- 0,0046 | 10,6071 +/- 1,1800 | 0,0297 +/- 0,0007 |
| `EXP4_RESNET18_RGBD` | `ResNet18Grasp` | RGB-D | 0,6492 +/- 0,0102 | 0,3879 +/- 0,0148 | 11,4260 +/- 0,5339 | 0,0283 +/- 0,0006 |

El mejor resultado medio corresponde a `EXP3_RESNET18_RGB_AUGMENT`, con `val_success` de 0,6801. La mejora frente a la CNN ligera RGB (`EXP1`) es amplia, y la comparacion frente a `EXP4` indica que ResNet-18 con RGB y augmentation supera en success medio a la variante ResNet-18 RGB-D sin augmentation. No obstante, `EXP4` mantiene la menor perdida de validacion media, lo que sugiere que diferentes metricas pueden favorecer lecturas distintas del rendimiento.

En la familia ligera, `EXP2_SIMPLE_RGBD` mejora el success de `EXP1_SIMPLE_RGB`, aunque conserva un error angular medio ligeramente superior. Este comportamiento refuerza que la profundidad y la augmentation no deben interpretarse como mejoras universales, sino como factores dependientes de arquitectura, preprocesamiento y metrica de evaluacion.

## Discusion

Los resultados apoyan tres conclusiones principales. Primero, la arquitectura basada en ResNet-18 ofrece una ventaja clara frente a la CNN ligera bajo el protocolo evaluado. Segundo, la modalidad RGB-D no domina de forma uniforme: aporta mejora en la CNN ligera, pero no supera a RGB con augmentation en la comparacion ResNet. Tercero, la reproducibilidad del analisis depende tanto de conservar las metricas por semilla como de declarar con precision la arquitectura real instanciada por cada configuracion.

La contribucion metodologica del trabajo reside en hacer explicita la cadena completa de decisiones experimentales: configuracion, particion, modalidad, arquitectura, semillas, criterio de perdida, metrica y agregacion. Esta estructura permite separar resultados oficiales, variantes auxiliares y extensiones metodologicas posteriores. Tambien reduce el riesgo de sobredimensionar conclusiones a partir de un conjunto experimental limitado.

Desde el punto de vista aplicado, los resultados identifican `EXP3_RESNET18_RGB_AUGMENT` como configuracion preferente para escenarios donde se prioriza el exito de agarre validado offline. La integracion en sistemas de inferencia puede documentarse como evidencia contextual, pero no constituye la prueba principal del articulo ni sustituye una validacion fisica sistematica.

## Limitaciones

- La evaluacion principal se realiza sobre Cornell; no demuestra generalizacion universal a otros datasets ni a escenas reales arbitrarias.
- Cada configuracion dispone de tres semillas, suficientes para trazabilidad comparativa pero limitadas para inferencia estadistica fuerte.
- Las metricas oficiales proceden del evaluador historico; la variante con IoU orientada y perdida angular debe evaluarse en un bloque experimental separado.
- La comparacion RGB frente a RGB-D esta condicionada por el preprocesamiento disponible y por la combinacion concreta de augmentation y arquitectura.
- La evidencia robotica debe entenderse como contexto de aplicacion, no como validacion experimental principal.

## Conclusiones

El articulo presenta una comparacion reproducible de arquitecturas CNN para deteccion de agarres 2D/2.5D en entornos no estructurados. Bajo el protocolo oficial, `ResNet18Grasp` con entrada RGB y augmentation obtiene el mejor success medio, mientras que la CNN ligera actua como baseline interpretable de baja complejidad. La principal aportacion es metodologica: ordenar los artefactos experimentales, separar configuraciones oficiales y auxiliares, y proporcionar una lectura verificable de los efectos de arquitectura, modalidad de entrada y augmentation.

## Trabajo futuro

- Reentrenar el bloque oficial con perdida angular periodica e IoU orientada.
- Extender la comparacion a datasets como Jacquard, GraspNet o Acronym.
- Incluir arquitecturas densas que predigan mapas de calidad, angulo y apertura.
- Ampliar el numero de semillas y realizar ablations especificas de augmentation y profundidad.
- Validar la configuracion seleccionada en entornos fisicos o simulaciones con mayor diversidad visual.

## Referencias preliminares

Pendiente de completar con bibliografia formal. Candidatas: Cornell Grasping Dataset; GGCNN; Jacquard; Dex-Net; GraspNet; ResNet.
