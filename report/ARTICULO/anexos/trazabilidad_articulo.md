<!-- generado-articulo-tfm -->

# Trazabilidad de afirmaciones del articulo

| Afirmacion | Fuente interna | Estado |
|---|---|---|
| Existen tres familias implementadas actuales: SimpleCNN, ResNetGrasp y SimpleGrasp. | `agarre_inteligente/src/models/*.py`, `src/models/factory.py` | verificado |
| `SimpleGraspCNN` instancia `SimpleCNN`. | `agarre_inteligente/src/models/factory.py` | verificado |
| `ResNet18Grasp` instancia `ResNetGrasp`. | `agarre_inteligente/src/models/factory.py` | verificado |
| Los experimentos oficiales son EXP1..EXP4. | `agarre_inteligente/README.md`, `report/metrics/validated/chapter5_experiment_summary_validated.csv` | verificado |
| EXP1.1/EXP1.2 son auxiliares y no sustituyen resultados oficiales. | `report/evidence/exp1_1_exp1_2_theoretical_implementation_trace_20260415.md`, `agarre_inteligente/README.md` | verificado |
| El preset de memoria fija EXP3 seed_0. | `agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_tfm_science.py` | verificado |
| Mejor modelo oficial por success medio: `EXP3_RESNET18_RGB_AUGMENT`. | `agarre_inteligente/experiments/EXP3_RESNET18_RGB_AUGMENT/best_epoch_summary.csv`, `report/metrics/validated/chapter5_experiment_summary_validated.csv` | verificado |
| Las metricas oficiales usan SmoothL1Loss y evaluador historico. | `agarre_inteligente/scripts/train.py`, `agarre_inteligente/src/evaluation/evaluator.py` | verificado |
| Existe variante metodologica posterior con GraspLoss/EvaluatorOriented. | `agarre_inteligente/src/training/losses.py`, `agarre_inteligente/src/evaluation/evaluator.py`, `config/exp_methodology_v2.yaml` | verificado como configuracion, no resultado oficial |
