# Trazabilidad de afirmaciones del articulo

| Afirmacion | Fuente interna | Estado |
|---|---|---|
| El articulo compara arquitecturas CNN para agarres 2D/2.5D con entradas RGB y RGB-D. | `agarre_inteligente/config/*.yaml`, `agarre_inteligente/src/models/*.py` | verificado |
| `SimpleGraspCNN` instancia `SimpleCNN`. | `agarre_inteligente/src/models/factory.py` | verificado |
| `ResNet18Grasp` instancia `ResNetGrasp`. | `agarre_inteligente/src/models/factory.py` | verificado |
| Los experimentos oficiales del articulo son `EXP1..EXP4`. | `recursos/tablas/chapter5_experiment_summary_validated.csv` | verificado |
| `EXP1.1/EXP1.2` son complementarios dentro del articulo y no sustituyen resultados oficiales. | `report/evidence/exp1_1_exp1_2_theoretical_implementation_trace_20260415.md`, `recursos/tablas/resumen_metricas_por_experimento.csv` | verificado |
| Mejor configuracion oficial por success medio: `EXP3_RESNET18_RGB_AUGMENT`. | `recursos/tablas/chapter5_experiment_summary_validated.csv` | verificado |
| Las metricas oficiales usan entrenamiento con `SmoothL1Loss` y evaluador historico. | `agarre_inteligente/scripts/train.py`, `agarre_inteligente/src/evaluation/evaluator.py` | verificado |
| Existe variante metodologica posterior con `GraspLoss`/evaluacion orientada. | `agarre_inteligente/src/training/losses.py`, `agarre_inteligente/src/evaluation/evaluator.py`, `agarre_inteligente/config/exp_methodology_v2.yaml` | verificado como configuracion, no resultado oficial |

## Regla editorial

Las afirmaciones cuantitativas del bloque principal deben salir de `recursos/tablas/chapter5_experiment_summary_validated.csv` cuando se hable de `EXP1..EXP4`. Los resultados `EXP1.1/EXP1.2` pueden aparecer en el cuerpo del articulo como bloque complementario, citando `recursos/tablas/resumen_metricas_por_experimento.csv` y dejando claro que no sustituyen el nucleo oficial.
