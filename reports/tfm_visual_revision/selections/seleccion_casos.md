# Seleccion curada de casos para comentarios 2, 3 y 4

## Criterio general de seleccion

La seleccion se ha realizado a partir del inventario reproducible generado en:

- `inventory/qualitative_inventory_combined.csv`
- `intermediate/overlays/`

Se han priorizado los casos que cumplen simultaneamente estos criterios:

- legibilidad alta en impresion;
- objetos reconocibles por un tribunal sin necesidad de contexto adicional;
- contraste suficiente entre GT y Pred;
- diversidad visual moderada, evitando repeticiones casi identicas;
- coherencia con el comentario del tutor sobre separar aciertos y fallos;
- trazabilidad exacta hasta imagen original, experimento y overlay individual.

## Aciertos seleccionados para la nueva figura de aciertos

Todos los aciertos se han tomado de `EXP3_RESNET18_RGB_AUGMENT` (seed 0), por ser la seed con mejor `val_success` dentro de ese experimento y por ofrecer ejemplos mas limpios visualmente.

### Caso A

- Imagen: `data/raw/cornell/02/pcd0254r.png`
- Overlay fuente:
  - `intermediate/overlays/EXP3_RESNET18_RGB_AUGMENT_seed_0/data__raw__cornell__02__pcd0254r.png`
- Metricas:
  - IoU = 0.75
  - dtheta = 10.6 deg
- Justificacion:
  - ejemplo compacto y limpio;
  - buen solape y orientacion coherente;
  - objeto facil de interpretar.

### Caso B

- Imagen: `data/raw/cornell/02/pcd0284r.png`
- Overlay fuente:
  - `intermediate/overlays/EXP3_RESNET18_RGB_AUGMENT_seed_0/data__raw__cornell__02__pcd0284r.png`
- Metricas:
  - IoU = 0.72
  - dtheta = 6.5 deg
- Justificacion:
  - objeto alargado que permite apreciar bien la coincidencia geometrica;
  - complementa al caso A con una morfologia distinta.

### Caso C

- Imagen: `data/raw/cornell/06/pcd0611r.png`
- Overlay fuente:
  - `intermediate/overlays/EXP3_RESNET18_RGB_AUGMENT_seed_0/data__raw__cornell__06__pcd0611r.png`
- Metricas:
  - IoU = 0.67
  - dtheta = 3.8 deg
- Justificacion:
  - orientacion especialmente precisa;
  - aporta variedad frente a objetos menos rectangulares.

### Caso D

- Imagen: `data/raw/cornell/06/pcd0691r.png`
- Overlay fuente:
  - `intermediate/overlays/EXP3_RESNET18_RGB_AUGMENT_seed_0/data__raw__cornell__06__pcd0691r.png`
- Metricas:
  - IoU = 0.68
  - dtheta = 21.2 deg
- Justificacion:
  - sigue siendo acierto Cornell, pero con discrepancia angular mayor que los demas;
  - ayuda a mostrar que un acierto no exige coincidencia perfecta, sino cumplimiento simultaneo de umbrales.

## Fallos seleccionados para la nueva figura de fallos E1-E4

La seleccion mezcla `EXP1_SIMPLE_RGB` y `EXP3_RESNET18_RGB_AUGMENT` porque la prioridad aqui no es comparar modelos, sino ilustrar con nitidez una tipologia de fallo. En cada caso se ha escogido el ejemplo mas explicativo dentro de los candidatos revisados.

### E1

- Imagen: `data/raw/cornell/02/pcd0287r.png`
- Experimento: `EXP1_SIMPLE_RGB`
- Overlay fuente:
  - `intermediate/overlays/EXP1_SIMPLE_RGB_seed_0/data__raw__cornell__02__pcd0287r.png`
- Metricas:
  - IoU = 0.80
  - dtheta = 52.1 deg
- Justificacion:
  - el solape es alto, pero la orientacion incumple claramente el umbral;
  - ejemplo especialmente didactico para un error angular puro.

### E2

- Imagen: `data/raw/cornell/02/pcd0268r.png`
- Experimento: `EXP1_SIMPLE_RGB`
- Overlay fuente:
  - `intermediate/overlays/EXP1_SIMPLE_RGB_seed_0/data__raw__cornell__02__pcd0268r.png`
- Metricas:
  - IoU = 0.23
  - dtheta = 30.9 deg
- Justificacion:
  - la prediccion se situa sobre la zona funcional del objeto, pero el solape es insuficiente;
  - el ejemplo visualiza bien el caso de "casi acierto" que no supera el criterio Cornell.

### E3

- Imagen: `data/raw/cornell/06/pcd0633r.png`
- Experimento: `EXP3_RESNET18_RGB_AUGMENT`
- Overlay fuente:
  - `intermediate/overlays/EXP3_RESNET18_RGB_AUGMENT_seed_0/data__raw__cornell__06__pcd0633r.png`
- Metricas:
  - IoU = 0.23
  - dtheta = 28.1 deg
- Razon geometrica adicional:
  - `area_ratio_pred_gt = 0.23`
- Justificacion:
  - la caja predicha es demasiado pequena para la apertura plausible del objeto;
  - ejemplo muy claro para ilustrar un error de tamano/apertura.

### E4

- Imagen: `data/raw/cornell/02/pcd0250r.png`
- Experimento: `EXP1_SIMPLE_RGB`
- Overlay fuente:
  - `intermediate/overlays/EXP1_SIMPLE_RGB_seed_0/data__raw__cornell__02__pcd0250r.png`
- Metricas:
  - IoU = 0.00
  - dtheta = 26.0 deg
- Razon geometrica adicional:
  - `center_distance_norm = 3.24`
- Justificacion:
  - la prediccion cae claramente sobre el fondo y no sobre el objeto;
  - visualmente es el ejemplo mas limpio de confusion fuera de la region util.

## Caso ilustrativo para el comentario 4

- Imagen: `data/raw/cornell/09/pcd0914r.png`
- Experimento: `EXP3_RESNET18_RGB_AUGMENT`
- Overlay fuente:
  - `intermediate/overlays/EXP3_RESNET18_RGB_AUGMENT_seed_0/data__raw__cornell__09__pcd0914r.png`
- Metricas:
  - IoU = 0.76
  - dtheta = 8.7 deg
- Justificacion:
  - el objeto ocupa una region pequena dentro de la escena completa;
  - funciona bien en formato "imagen completa + recorte ampliado";
  - responde de forma directa a la observacion del tutor sobre ilustrar este fenomeno con un caso concreto.

## Destino en las figuras finales

- Figura de aciertos:
  - casos A-D
- Figura de fallos:
  - E1, E2, E3, E4
- Caso ilustrativo:
  - `pcd0914r`
