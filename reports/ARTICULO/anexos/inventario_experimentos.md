<!-- generado-articulo-tfm -->

# Inventario de experimentos

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

## Estados usados

- `final`: caso documentado para inferencia reproducible del articulo/TFM.
- `oficial`: experimento del bloque consolidado `EXP1..EXP4`.
- `auxiliar`: experimento posterior que implementa la arquitectura teorica `SimpleGrasp`.
- `experimental metodologico configurado`: YAML localizado sin resultados ejecutados en `agarre_inteligente/experiments`.
- `plantilla`: configuracion base no interpretable como resultado experimental.
