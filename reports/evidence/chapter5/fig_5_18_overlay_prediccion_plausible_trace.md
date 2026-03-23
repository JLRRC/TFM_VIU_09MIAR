# Figura 5-18 (overlay prediccion plausible, estilo TFM)

- Objeto objetivo: `pick_demo` (exclusivo).
- Imagen base: `/home/laboratorio/TFM/agarre_ros2_ws/log/panel_infer/frame_20260323_231409.png`
- Frame de inferencia usado: `/home/laboratorio/TFM/agarre_ros2_ws/log/panel_infer/frame_20260323_231409.png`
- Overlay panel (baseline): `/home/laboratorio/TFM/auditoria/panel_audit/figures/overlay_infer_1774304049745.png`
- Criterio de selección del caso: maximizar IoU y minimizar error angular/centroides respecto al GT del mismo frame.
- Baseline panel (seed_2): IoU=0.118067, dTheta=18.652244 deg.
- Candidato seleccionado: `/home/laboratorio/TFM/agarre_inteligente/experiments/EXP3_RESNET18_RGB_AUGMENT/seed_1/checkpoints/best.pth` (model=resnet18, in_ch=3).
- Métricas del caso seleccionado: IoU=0.146023, dTheta=8.243625 deg, dist_centro=3.928 px.
- Convención visual aplicada:
  - `GT`: verde discontinuo
  - `PRED`: rojo continuo
- Contexto recortado (solo objeto): x1=89, y1=99, x2=231, y2=240
- Zoom final: x1=125, y1=135, x2=195, y2=207
