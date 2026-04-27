# Figura 5-18 (overlay prediccion plausible, estilo TFM)

- Objeto objetivo: `pick_demo` (exclusivo).
- Imagen base: `/home/laboratorio/TFM/agarre_ros2_ws/log/panel_infer/frame_20260405_115602.png`
- Frame de inferencia usado: `/home/laboratorio/TFM/agarre_ros2_ws/log/panel_infer/frame_20260405_115602.png`
- Overlay panel (baseline): `/home/laboratorio/TFM/auditoria/panel_audit/figures/overlay_infer_43030453.png`
- Criterio de seleccion del caso: priorizar alineacion visual del centro (dist<= 5.0px y dTheta<= 15.0deg cuando exista), usando similitud de tamano como desempate.
- Baseline panel (seed_2): IoU=0.114745, dTheta=9.055633 deg.
- Candidato seleccionado: `/home/laboratorio/TFM/agarre_inteligente/experiments/EXP3_RESNET18_RGB_AUGMENT/seed_1/checkpoints/best.pth` (model=resnet18, in_ch=3).
- Metricas del caso seleccionado: IoU=0.146254, dTheta=5.805864 deg, dist_centro=3.194 px, ratio_w=0.448, ratio_h=0.327, size_err=0.613.
- Visualizacion PRED: adjusted=true raw_wh=(7.850,6.248) draw_wh=(16.663,18.164) min_req_wh=(16.663,18.164)
- Convención visual aplicada:
  - `GT`: verde discontinuo
  - `PRED`: rojo continuo (tamano minimo visual para cobertura del objeto)
- Contexto recortado (solo objeto): x1=86, y1=99, x2=231, y2=240
- Zoom final: x1=122, y1=135, x2=195, y2=209
