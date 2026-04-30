<!-- generado-articulo-tfm -->

# Titulo

Comparacion reproducible de CNN ligeras y ResNet-18 para prediccion de agarres 2D/2.5D con integracion ROS 2

## Resumen

Este articulo deriva de un trabajo de fin de master centrado en la deteccion de poses de agarre mediante vision y aprendizaje profundo. Se comparan modelos convolucionales para regresion de rectangulos de agarre en formato Cornell, usando imagenes RGB y RGB-D, particion object-wise y tres semillas por configuracion. El pipeline incluye entrenamiento, evaluacion, benchmark de latencia e integracion funcional en ROS 2/Gazebo/MoveIt 2 para un UR5 con pinza RG2. Los resultados oficiales muestran que `ResNet18Grasp` con RGB y augmentation (`EXP3_RESNET18_RGB_AUGMENT`) alcanza el mejor rendimiento medio entre los experimentos consolidados: `val_success` 0.6801 +/- 0.0300, IoU 0.4010 +/- 0.0057 y error angular 10.6071 +/- 1.4451 grados. El trabajo tambien documenta discrepancias entre la arquitectura ligera teorica y la baseline historica usada en resultados, preservando la trazabilidad entre codigo, configuraciones y artefactos experimentales.

## Palabras clave

agarre robotico; CNN; ResNet-18; RGB-D; Cornell grasping dataset; ROS 2; reproducibilidad

## Introduccion

La prediccion de agarres a partir de imagenes es una tarea central en manipulacion robotica. En escenarios no estructurados, una formulacion frecuente consiste en estimar un rectangulo orientado de agarre definido por centro, dimensiones y angulo. Este enfoque reduce la complejidad frente a formulaciones 6-DoF completas y permite evaluar modelos mediante protocolos derivados del Cornell Grasping Dataset.

El proyecto del que deriva este articulo implementa un pipeline completo: preparacion de datos, definicion de modelos, entrenamiento multisemilla, evaluacion, analisis de latencia e integracion de inferencia en un entorno ROS 2/Gazebo/MoveIt 2. La pregunta principal no es proponer una arquitectura nueva, sino medir de forma trazable que configuracion resulta mas adecuada dentro de un sistema robotico reproducible.

## Estado del arte

Pendiente de cerrar con referencias formales del venue. La redaccion deberia cubrir, como minimo:

- formulacion Cornell de rectangulos de agarre;
- redes densas tipo GGCNN y variantes RGB-D;
- enfoques basados en CNN profundas y transferencia;
- datasets Cornell, Jacquard y GraspNet/Acronym como contexto;
- integracion de percepcion con ROS/MoveIt para manipulacion.

## Metodologia

El pipeline experimental usa configuraciones YAML versionadas en `agarre_inteligente/config/`. Cada experimento define modelo, modalidad de entrada, augmentation, optimizador, epocas, batch size, funcion de perdida y rutas de datos. El entrenamiento se ejecuta mediante `scripts/train.py`, que construye el modelo desde `src/models/factory.py`, carga el dataset Cornell procesado, aplica transformaciones de entrenamiento/validacion y guarda metricas por epoca, checkpoints y `config_snapshot.yaml`.

La evaluacion oficial usa `SmoothL1Loss` para entrenamiento y metricas Cornell implementadas en el codigo historico. Existe una variante metodologica posterior con `GraspLoss` e IoU orientada, pero no sustituye los resultados oficiales.

## Modelos

| nombre_codigo | clase_real | archivo_definicion | arquitectura | entradas | salidas | uso_real | estado | evidencia |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| SimpleGraspCNN | SimpleCNN | agarre_inteligente/src/models/simple_cnn.py | CNN ligera: tres bloques Conv2d 3x3 + BatchNorm + ReLU + MaxPool, AdaptiveAvgPool2d(1,1), MLP 128-128-5. | Tensor imagen 224x224 con 3 canales RGB o 4 canales RGB-D segun input_channels. | Vector de 5 parametros: cx, cy, w, h, angle. En dataset se entrenan normalizados; el wrapper ROS decodifica a pixeles/angulo. | Entrenamiento oficial EXP1/EXP2, inferencia CLI generica, cargable por wrapper ROS actual. | Activo como baseline oficial; el nombre de clase no coincide con el nombre de configuracion. | factory.py construye SimpleCNN si model.name == SimpleGraspCNN; configs EXP1/EXP2; metrics en experiments/EXP1 y EXP2. |
| ResNet18Grasp | ResNetGrasp | agarre_inteligente/src/models/resnet_variants.py | torchvision ResNet-18 con fc reemplazada por Dropout + Linear(...,5); conv1 adaptada cuando input_channels != 3. | Tensor imagen 224x224 RGB (3 canales) o RGB-D (4 canales). | Vector de 5 parametros: cx, cy, w, h, angle. | Entrenamiento oficial EXP3/EXP4, inferencia CLI generica y wrapper ROS. El preset de memoria usa EXP3 seed_0. | Activo y modelo final documentado para inferencia reproducible. | factory.py construye ResNetGrasp para ResNet18Grasp; panel_tfm_science fija EXP3_RESNET18_RGB_AUGMENT/seed_0 en modo memoria. |
| SimpleGrasp | SimpleGrasp | agarre_inteligente/src/models/simple_grasp.py | CNN ligera alineada con diseno teorico: primera Conv2d 7x7 stride 2, bloques 3x3, AdaptiveAvgPool2d(7,7), MLP 128*7*7-256-5. | Tensor imagen 224x224 RGB o RGB-D segun input_channels. | Vector de 5 parametros normalizados; wrapper ROS aplica clipping para evitar decodificacion divergente. | Experimentos auxiliares EXP1.1/EXP1.2 y pruebas de carga del wrapper ROS. | Activo como variante auxiliar/posterior; no sustituye resultados oficiales EXP1..EXP4. | configs exp1_1/exp1_2, docs/modelos/simplegrasp.md, test_model_load_exp11.py. |
| graspnet.models.simple_grasp_cnn.SimpleGraspCNN | No encontrada en el arbol actual | No existe bajo agarre_inteligente/graspnet/models en el workspace actual | Referencia legacy no materializada como archivo actual. | El nodo legacy la invoca como RGB de 3 canales. | Vector de 5 parametros segun nodo legacy. | Nodo ROS legacy grasp_inference.py intenta importarla; el wrapper actual usa fallback a src/models. | Legacy/obsoleto o referencia rota; no debe presentarse como modelo implementado actual. | agarre_ros2_ws/src/tfm_grasping/tfm_grasping/grasp_inference.py importa graspnet.models.simple_grasp_cnn; find no localiza ese modulo. |

## Protocolo experimental

Los experimentos oficiales son `EXP1..EXP4`. Cada uno se ejecuta con tres seeds. Las metricas principales son `val_success`, `val_iou`, `val_angle_deg` y `val_loss` en la mejor epoca de validacion. Tambien hay mediciones de latencia CPU/CUDA para batch 1.

| experimento | modelo_config | clase_real | modalidad | canales | augmentation | augmentation_level | epochs | criterion | config_file | metrics_csv_por_seed | best_epoch_summary | estado |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| EXP1_SIMPLE_RGB | SimpleGraspCNN | SimpleCNN | RGB | 3 | False | -- | 10 | smooth_l1 | agarre_inteligente/config/exp1_simple_rgb.yaml | 3 | agarre_inteligente/experiments/EXP1_SIMPLE_RGB/best_epoch_summary.csv | oficial |
| EXP2_SIMPLE_RGBD | SimpleGraspCNN | SimpleCNN | RGB-D | 4 | True | moderate | 10 | smooth_l1 | agarre_inteligente/config/exp2_simple_rgbd.yaml | 3 | agarre_inteligente/experiments/EXP2_SIMPLE_RGBD/best_epoch_summary.csv | oficial |
| EXP3_RESNET18_RGB_AUGMENT | ResNet18Grasp | ResNetGrasp | RGB | 3 | True | moderate_strong | 50 | smooth_l1 | agarre_inteligente/config/exp3_resnet18_rgb_augment.yaml | 3 | agarre_inteligente/experiments/EXP3_RESNET18_RGB_AUGMENT/best_epoch_summary.csv | final |
| EXP4_RESNET18_RGBD | ResNet18Grasp | ResNetGrasp | RGB-D | 4 | False | -- | 50 | smooth_l1 | agarre_inteligente/config/exp4_resnet18_rgbd.yaml | 3 | agarre_inteligente/experiments/EXP4_RESNET18_RGBD/best_epoch_summary.csv | oficial |
| EXP1.1_SIMPLEGRASP_RGB | SimpleGrasp | SimpleGrasp | RGB | 3 | False | -- | 10 | smooth_l1 | agarre_inteligente/config/exp1_1_simplegrasp_rgb.yaml | 3 | agarre_inteligente/experiments/EXP1.1_SIMPLEGRASP_RGB/best_epoch_summary.csv | auxiliar |
| EXP1.2_SIMPLEGRASP_RGBD | SimpleGrasp | SimpleGrasp | RGB-D | 4 | True | moderate | 10 | smooth_l1 | agarre_inteligente/config/exp1_2_simplegrasp_rgbd.yaml | 3 | agarre_inteligente/experiments/EXP1.2_SIMPLEGRASP_RGBD/best_epoch_summary.csv | auxiliar |
| EXP_METHOD_V2_RGB | SimpleGraspCNN | SimpleCNN | RGB | 3 | False | -- | 10 | grasp_loss | agarre_inteligente/config/exp_methodology_v2.yaml | 0 | no localizado | experimental metodologico configurado |
| EXP_TEMPLATE | SimpleGraspCNN | SimpleCNN | RGB | 3 | False | -- | 10 | smooth_l1 | agarre_inteligente/config/default.yaml | 0 | no localizado | plantilla |

## Resultados

| experimento | modelo | modalidad | estado | success | IoU | error angular | val_loss |
| --- | --- | --- | --- | --- | --- | --- | --- |
| EXP1.1_SIMPLEGRASP_RGB | SimpleGrasp | RGB | auxiliar | 0.3646 +/- 0.1014 | 0.2477 +/- 0.0808 | 14.5377 +/- 1.5389 | 0.0336 +/- 0.0024 |
| EXP1.2_SIMPLEGRASP_RGBD | SimpleGrasp | RGB-D | auxiliar | 0.4143 +/- 0.0404 | 0.3079 +/- 0.0517 | 17.1200 +/- 0.2046 | 0.0334 +/- 0.0006 |
| EXP1_SIMPLE_RGB | SimpleGraspCNN | RGB | oficial | 0.2741 +/- 0.0539 | 0.2254 +/- 0.0608 | 17.1910 +/- 1.6834 | 0.0321 +/- 0.0010 |
| EXP2_SIMPLE_RGBD | SimpleGraspCNN | RGB-D | oficial | 0.3805 +/- 0.0063 | 0.3169 +/- 0.0041 | 17.8730 +/- 0.4418 | 0.0320 +/- 0.0002 |
| EXP3_RESNET18_RGB_AUGMENT | ResNet18Grasp | RGB | final | 0.6801 +/- 0.0300 | 0.4010 +/- 0.0057 | 10.6071 +/- 1.4451 | 0.0297 +/- 0.0008 |
| EXP4_RESNET18_RGBD | ResNet18Grasp | RGB-D | oficial | 0.6492 +/- 0.0125 | 0.3879 +/- 0.0182 | 11.4260 +/- 0.6538 | 0.0283 +/- 0.0008 |

El mejor resultado medio entre los experimentos ejecutados corresponde a `EXP3_RESNET18_RGB_AUGMENT` con `val_success` 0.6801. En el bloque oficial del TFM, el caso final documentado para inferencia es `EXP3_RESNET18_RGB_AUGMENT`.

## Discusion

Los resultados indican que la familia ResNet-18 proporciona una mejora clara frente a la CNN ligera oficial. La variante RGB con augmentation (`EXP3`) supera a la configuracion RGB-D de ResNet (`EXP4`) en success medio, aunque `EXP4` conserva un `val_loss` competitivo. En la CNN ligera, RGB-D con augmentation (`EXP2`) mejora a RGB sin augmentation (`EXP1`), lo que sugiere que la utilidad de profundidad y augmentation depende de arquitectura y protocolo.

El articulo debe explicar que `SimpleGraspCNN` es un nombre de configuracion: la clase real en el codigo actual es `SimpleCNN`. Tambien debe separar los resultados oficiales de `EXP1..EXP4` de `EXP1.1/EXP1.2`, que implementan la arquitectura ligera teorica `SimpleGrasp`.

## Limitaciones

- Dataset Cornell y validacion offline; no demuestra generalizacion universal.
- Tres semillas por experimento; analisis estadistico limitado.
- Metricas oficiales historicas no usan la variante orientada posterior.
- Integracion robotica funcional en simulacion; no se documenta validacion fisica en robot real.
- La profundidad no produce una mejora uniforme en todos los modelos.

## Conclusiones

El proyecto ofrece una comparacion reproducible y trazable de modelos CNN para prediccion de agarres 2D/2.5D. La mejor configuracion oficial es `ResNet18Grasp` RGB con augmentation. La infraestructura conserva codigo, configuraciones, metricas, checkpoints e integracion ROS 2 suficientes para construir un articulo centrado en reproducibilidad experimental e integracion aplicada.

## Trabajo futuro

- Evaluar con IoU orientada y perdida angular periodica de forma oficial.
- Ampliar datasets y protocolos de validacion.
- Validar en robot fisico o con mayor diversidad de escenas simuladas.
- Comparar con modelos densos de mapas de calidad/angulo/apertura.
- Reducir latencia y tamano de modelos para ejecucion embarcada.

## Referencias preliminares

Pendiente de completar con bibliografia formal. Candidatas: Cornell Grasping Dataset; GGCNN; Dex-Net; Jacquard; GraspNet; MoveIt 2; ROS 2; ResNet.
