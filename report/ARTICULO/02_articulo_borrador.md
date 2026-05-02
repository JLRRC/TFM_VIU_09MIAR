# Titulo

HACIA UN GRASPING 2D/2.5D REPRODUCIBLE: COMPARACIÓN DE ARQUITECTURAS CNN EN ENTORNOS NO ESTRUCTURADOS

## Resumen

El grasping visual en entornos no estructurados requiere convertir imagenes RGB o RGB-D en parametros geometricos de agarre precisos, comparables y accionables. Este articulo presenta una comparacion reproducible de arquitecturas CNN para deteccion de agarres 2D/2.5D mediante regresion de rectangulos en una representacion tipo Cornell. Se comparan dos familias principales: una CNN ligera configurada como `SimpleGraspCNN` e implementada como `SimpleCNN`, y una arquitectura residual `ResNet18Grasp` implementada sobre ResNet-18. El protocolo fija particion object-wise, configuraciones YAML versionadas, tres semillas por experimento, entrenamiento con PyTorch y evaluacion mediante `grasp success`, IoU, error angular y perdida de validacion. Los resultados oficiales muestran que `ResNet18Grasp` con entrada RGB y augmentation (`EXP3_RESNET18_RGB_AUGMENT`) obtiene el mejor rendimiento medio: `val_success` 0,6801 +/- 0,0245, IoU 0,4010 +/- 0,0046 y error angular 10,6071 +/- 1,1800 grados. La comparacion evidencia que la arquitectura residual supera a la CNN ligera, mientras que RGB-D no proporciona una mejora uniforme frente a RGB con augmentation. La aportacion principal es metodologica y experimental: un estudio controlado sobre arquitectura, modalidad de entrada, augmentation, precision geometrica, eficiencia y aplicabilidad funcional final.

## Palabras clave

grasping 2D/2.5D; redes convolucionales; ResNet-18; RGB-D; Cornell grasping dataset; reproducibilidad

## Introduccion

En escenas no estructuradas, detectar un agarre no equivale solo a localizar un objeto: implica estimar una geometria ejecutable con centro, dimensiones y orientacion coherentes. En grasping 2D/2.5D, esta formulacion permite transformar informacion visual en rectangulos de agarre evaluables y potencialmente ejecutables con pinzas paralelas. La dificultad aparece cuando pequenas diferencias de arquitectura, modalidad de entrada o entrenamiento cambian la lectura del rendimiento.

El interes de este estudio surge de una tension frecuente en aprendizaje profundo aplicado: los resultados dependen tanto del modelo como del protocolo experimental. La arquitectura elegida, la modalidad RGB o RGB-D, el uso de augmentation, la particion del dataset, las semillas y las metricas pueden modificar sustancialmente la conclusion. Sin un protocolo controlado, la comparacion entre modelos pierde fuerza y puede confundir ventajas arquitectonicas con efectos del procedimiento.

Este articulo aborda esa brecha mediante una comparacion reproducible de arquitecturas CNN para deteccion de agarres 2D/2.5D. El foco no es proponer una arquitectura nueva ni reclamar superioridad universal, sino mostrar que se aprende al comparar una CNN ligera y una ResNet-18 adaptada bajo el mismo marco experimental. La contribucion se articula en tres planos: rigor metodologico, resultados comparativos y cierre funcional de aplicabilidad.

## Estado del arte y posicionamiento

La representacion rectangular de agarres, popularizada por el Cornell Grasping Dataset [1], ofrece una forma compacta de evaluar poses de agarre para pinzas paralelas. Frente a enfoques 6-DoF mas generales, el grasping 2D/2.5D permite concentrarse en la relacion entre imagen, geometria proyectada y exito de una configuracion de agarre. Esta simplificacion sigue siendo util para estudiar arquitecturas de vision y protocolos de evaluacion.

El campo ha evolucionado desde detectores rectangulares sobre datasets acotados hacia metodos densos, reactivos o basados en grandes benchmarks. GG-CNN muestra el interes de predicciones densas en tiempo real para control en bucle cerrado [2]. Jacquard amplia la escala de datos sinteticos RGB-D para deteccion de agarres [3], Dex-Net 2.0 explota datos sinteticos y metricas analiticas para aprender calidad de agarre desde profundidad [4], y GraspNet-1Billion desplaza la evaluacion hacia benchmarks RGB-D de gran escala y agarres 6-DoF en escenas con desorden [5]. Este articulo se situa deliberadamente antes de esa complejidad: compara de forma controlada arquitecturas CNN para agarres 2D/2.5D sobre una representacion tipo Cornell, sin presentarse como un benchmark universal.

Las CNN ligeras resultan atractivas por su bajo coste y facilidad de despliegue, pero pueden limitar la extraccion de caracteristicas complejas. Las arquitecturas residuales, como ResNet-18 [6], ofrecen mayor capacidad representacional y transferencia visual, a costa de mas parametros. A su vez, la profundidad puede aportar informacion geometrica, aunque su impacto depende del preprocesamiento, de la arquitectura y de la metrica empleada. Por ello, una comparacion rigurosa debe controlar arquitectura, modalidad de entrada y augmentation.

El estudio se posiciona como una evaluacion reproducible y acotada. Su valor no esta en competir directamente con todos los metodos del estado del arte, sino en ofrecer una lectura clara, auditable y util sobre decisiones experimentales que afectan al rendimiento en grasping 2D/2.5D. Esta orientacion conecta con las recomendaciones de reproducibilidad en aprendizaje automatico y AI, que insisten en documentar datos, metodo, configuraciones, codigo y condiciones experimentales para que los resultados sean verificables [7], [8].

## Metodologia

El protocolo experimental se disena para que cada resultado pueda rastrearse desde la configuracion hasta la metrica final. Cada experimento se define mediante YAML, incluyendo arquitectura, modalidad de entrada, augmentation, optimizador, epocas, batch size, criterio de perdida y rutas de datos. El dataset Cornell procesado se divide con particion object-wise, evitando que instancias del mismo objeto aparezcan simultaneamente en entrenamiento y validacion.

Las entradas se normalizan a 224 x 224 pixeles. Las configuraciones RGB usan tres canales; las RGB-D incorporan un cuarto canal de profundidad. Las configuraciones con augmentation aplican transformaciones controladas como rotacion, volteo horizontal y variaciones de color cuando corresponde. La salida de los modelos es un vector de cinco parametros: `cx`, `cy`, `w`, `h` y `angle`.

La evaluacion oficial usa `SmoothL1Loss` y reporta `val_success`, `val_iou`, `val_angle_deg` y `val_loss` en la mejor epoca de validacion. Cada configuracion se ejecuta con tres semillas. Los resultados se agregan como media y desviacion entre semillas, y se preservan las metricas por epoca y los resumenes de mejor epoca. Existe una variante metodologica posterior con perdida angular periodica e IoU orientada, pero se mantiene como extension pendiente para no mezclar resultados consolidados con ensayos no equivalentes.

## Modelos evaluados

| modelo en configuracion | clase implementada | modalidad | descripcion | papel en el estudio |
| --- | --- | --- | --- | --- |
| `SimpleGraspCNN` | `SimpleCNN` | RGB / RGB-D | CNN ligera con bloques convolucionales, normalizacion, pooling adaptativo y MLP final de cinco salidas. | Baseline de baja complejidad. |
| `ResNet18Grasp` | `ResNetGrasp` | RGB / RGB-D | ResNet-18 con capa final adaptada a cinco parametros de agarre y primera convolucion ajustable a cuatro canales. | Arquitectura residual principal. |
| `SimpleGrasp` | `SimpleGrasp` | RGB / RGB-D | Variante ligera posterior entrenada en experimentos complementarios. | Bloque complementario dentro del articulo; no sustituye el bloque oficial. |

La distincion entre nombre de configuracion y clase real es importante: `SimpleGraspCNN` instancia `SimpleCNN`, mientras que `ResNet18Grasp` instancia `ResNetGrasp`.

## Protocolo experimental

El nucleo del articulo esta formado por cuatro experimentos oficiales:

| experimento | modelo_config | clase_real | modalidad | canales | augmentation | epocas | criterio | estado |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `EXP1_SIMPLE_RGB` | `SimpleGraspCNN` | `SimpleCNN` | RGB | 3 | no | 10 | `smooth_l1` | oficial |
| `EXP2_SIMPLE_RGBD` | `SimpleGraspCNN` | `SimpleCNN` | RGB-D | 4 | si | 10 | `smooth_l1` | oficial |
| `EXP3_RESNET18_RGB_AUGMENT` | `ResNet18Grasp` | `ResNetGrasp` | RGB | 3 | si | 50 | `smooth_l1` | principal |
| `EXP4_RESNET18_RGBD` | `ResNet18Grasp` | `ResNetGrasp` | RGB-D | 4 | no | 50 | `smooth_l1` | oficial |

Los experimentos `EXP1.1` y `EXP1.2` se incluyen en el articulo como bloque complementario de una variante ligera posterior. Su funcion es ampliar la trazabilidad de modelos ligeros, no reemplazar el nucleo comparativo `EXP1..EXP4`.

## Resultados

| experimento | modelo | modalidad | success | IoU | error angular | val_loss |
| --- | --- | --- | --- | --- | --- | --- |
| `EXP1_SIMPLE_RGB` | `SimpleGraspCNN` | RGB | 0,2741 +/- 0,0440 | 0,2254 +/- 0,0496 | 17,1910 +/- 1,3745 | 0,0321 +/- 0,0008 |
| `EXP2_SIMPLE_RGBD` | `SimpleGraspCNN` | RGB-D | 0,3805 +/- 0,0051 | 0,3169 +/- 0,0033 | 17,8730 +/- 0,3608 | 0,0320 +/- 0,0001 |
| `EXP3_RESNET18_RGB_AUGMENT` | `ResNet18Grasp` | RGB | 0,6801 +/- 0,0245 | 0,4010 +/- 0,0046 | 10,6071 +/- 1,1800 | 0,0297 +/- 0,0007 |
| `EXP4_RESNET18_RGBD` | `ResNet18Grasp` | RGB-D | 0,6492 +/- 0,0102 | 0,3879 +/- 0,0148 | 11,4260 +/- 0,5339 | 0,0283 +/- 0,0006 |

El mejor resultado medio corresponde a `EXP3_RESNET18_RGB_AUGMENT`, con `val_success` de 0,6801. La arquitectura residual mejora de forma clara a la CNN ligera en las modalidades evaluadas. En ResNet, la configuracion RGB con augmentation supera en success e IoU a la alternativa RGB-D sin augmentation, aunque `EXP4` mantiene la menor perdida de validacion. En la CNN ligera, RGB-D con augmentation mejora el success frente a RGB, pero no reduce el error angular medio.

Como bloque complementario, `EXP1.1_SIMPLEGRASP_RGB` y `EXP1.2_SIMPLEGRASP_RGBD` permiten contextualizar una variante ligera posterior:

| experimento | modelo | modalidad | success | IoU | error angular | val_loss |
| --- | --- | --- | --- | --- | --- | --- |
| `EXP1.1_SIMPLEGRASP_RGB` | `SimpleGrasp` | RGB | 0,3646 +/- 0,1014 | 0,2477 +/- 0,0808 | 14,5377 +/- 1,5389 | 0,0336 +/- 0,0024 |
| `EXP1.2_SIMPLEGRASP_RGBD` | `SimpleGrasp` | RGB-D | 0,4143 +/- 0,0404 | 0,3079 +/- 0,0517 | 17,1200 +/- 0,2046 | 0,0334 +/- 0,0006 |

Estos resultados no cambian la conclusion principal: la arquitectura residual sigue aportando el salto de rendimiento mas claro. Su valor editorial esta en mostrar que el estudio conserva trazabilidad sobre variantes ligeras posteriores sin mezclar su lectura con el bloque oficial.

Estos resultados muestran que la profundidad y el aumento de datos no deben tratarse como mejoras universales. Su efecto depende de la arquitectura y de la metrica usada para interpretar el rendimiento.

## Discusion

El estudio deja tres aprendizajes principales. Primero, la capacidad representacional de ResNet-18 aporta una ventaja clara sobre la CNN ligera bajo el protocolo evaluado. Segundo, RGB-D no domina automaticamente: puede ayudar a la arquitectura ligera, pero no supera a RGB con augmentation en la configuracion residual. Tercero, la comparacion solo es defendible porque se conserva la trazabilidad entre configuraciones, semillas, metricas por epoca y resumenes agregados.

La aportacion metodologica es tan importante como el resultado numerico. El estudio convierte un conjunto de entrenamientos en una comparacion auditable: se sabe que se compara, como se entreno, que metrica selecciona la mejor epoca y que limites afectan a la interpretacion. Esta claridad permite discutir precision, geometria del grasp y eficiencia sin sobredimensionar conclusiones.

## Aplicabilidad funcional

El modelo principal (`EXP3_RESNET18_RGB_AUGMENT`) queda identificado como candidato razonable para inferencia funcional cuando se prioriza `val_success`. Esta integracion aplicada funciona como cierre practico: muestra que la comparacion puede conectarse con un flujo de ejecucion, pero no sustituye una evaluacion fisica sistematica ni desplaza el foco experimental.

## Limitaciones

- La evaluacion principal se realiza sobre Cornell; no demuestra generalizacion universal.
- Cada configuracion dispone de tres semillas, suficientes para trazabilidad comparativa pero limitadas para inferencia estadistica fuerte.
- Las metricas oficiales proceden del evaluador historico; la variante con IoU orientada debe evaluarse en un bloque separado.
- La comparacion RGB/RGB-D esta condicionada por preprocesamiento, arquitectura y augmentation.
- La validacion funcional final es evidencia de aplicabilidad, no prueba de rendimiento fisico real.

## Conclusiones

Este articulo presenta un estudio reproducible sobre grasping 2D/2.5D en entornos no estructurados mediante comparacion controlada de arquitecturas CNN. Bajo el protocolo oficial, `ResNet18Grasp` con RGB y augmentation obtiene el mejor rendimiento medio, mientras que la CNN ligera aporta una baseline interpretable. El valor del trabajo reside en la combinacion de rigor experimental, lectura comparativa y cierre funcional: una base seria para extender la evaluacion hacia metricas orientadas, nuevos datasets, mas semillas y validacion fisica.

## Trabajo futuro

- Reentrenar el bloque oficial con perdida angular periodica e IoU orientada.
- Extender la comparacion a Jacquard, GraspNet, Acronym u otros datasets.
- Incluir arquitecturas densas que predigan mapas de calidad, angulo y apertura.
- Ampliar semillas y realizar ablations especificas de augmentation y profundidad.
- Incorporar analisis cualitativo de aciertos y fallos con ejemplos visuales seleccionados.
- Validar la configuracion seleccionada en escenarios fisicos o simulaciones mas diversas.

## Referencias

[1] I. Lenz, H. Lee, and A. Saxena, "Deep learning for detecting robotic grasps," *The International Journal of Robotics Research*, vol. 34, no. 4-5, pp. 705-724, 2015. doi: 10.1177/0278364914549607.

[2] D. Morrison, P. Corke, and J. Leitner, "Closing the Loop for Robotic Grasping: A Real-time, Generative Grasp Synthesis Approach," in *Robotics: Science and Systems XIV*, 2018. doi: 10.15607/RSS.2018.XIV.021.

[3] A. Depierre, E. Dellandrea, and L. Chen, "Jacquard: A Large Scale Dataset for Robotic Grasp Detection," in *2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, 2018, pp. 3511-3516. doi: 10.1109/IROS.2018.8593950.

[4] J. Mahler, J. Liang, S. Niyaz, M. Laskey, R. Doan, X. Liu, J. Aparicio Ojea, and K. Goldberg, "Dex-Net 2.0: Deep Learning to Plan Robust Grasps with Synthetic Point Clouds and Analytic Grasp Metrics," in *Robotics: Science and Systems XIII*, 2017. doi: 10.15607/RSS.2017.XIII.058.

[5] H.-S. Fang, C. Wang, M. Gou, and C. Lu, "GraspNet-1Billion: A Large-Scale Benchmark for General Object Grasping," in *Proceedings of the IEEE/CVF Conference on Computer Vision and Pattern Recognition (CVPR)*, 2020, pp. 11444-11453. doi: 10.1109/CVPR42600.2020.01146.

[6] K. He, X. Zhang, S. Ren, and J. Sun, "Deep Residual Learning for Image Recognition," in *Proceedings of the IEEE Conference on Computer Vision and Pattern Recognition (CVPR)*, 2016, pp. 770-778. doi: 10.1109/CVPR.2016.90.

[7] J. Pineau, P. Vincent-Lamarre, K. Sinha, V. Lariviere, A. Beygelzimer, F. d'Alche-Buc, E. Fox, and H. Larochelle, "Improving Reproducibility in Machine Learning Research (A Report from the NeurIPS 2019 Reproducibility Program)," *Journal of Machine Learning Research*, vol. 22, no. 164, pp. 1-20, 2021.

[8] O. E. Gundersen and S. Kjensmo, "State of the Art: Reproducibility in Artificial Intelligence," in *Proceedings of the AAAI Conference on Artificial Intelligence*, vol. 32, no. 1, pp. 1644-1651, 2018. doi: 10.1609/aaai.v32i1.11503.
