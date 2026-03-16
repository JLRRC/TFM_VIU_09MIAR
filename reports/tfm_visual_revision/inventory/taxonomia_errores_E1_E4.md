# Taxonomia de errores E1-E4 para la revision visual del TFM

## Referencia textual existente en el TFM

La taxonomia no se ha inventado para esta revision. Ya aparece en la seccion 5.2.1 del PDF `docs/TFM_V7.9.pdf`, con estas formulaciones base:

- E1 - Error angular
- E2 - Bajo solapamiento (IoU)
- E3 - Tamano/apertura no plausible
- E4 - Confusion por fondo u oclusion

## Definicion operativa usada en esta revision

### E1 - Error angular

Se asigna E1 cuando la prediccion queda cerca del objeto y el solape puede ser incluso alto, pero la orientacion incumple claramente el umbral angular del criterio Cornell.

Indicadores usados:

- IoU no necesariamente bajo;
- dtheta > 30 deg;
- prediccion visualmente "encima" del objeto.

### E2 - Bajo solapamiento (IoU)

Se asigna E2 cuando la prediccion apunta a la zona funcional correcta o al menos a una region proxima, pero no alcanza el solape requerido con el GT seleccionado.

Indicadores usados:

- IoU < 0.25 o muy cercano al umbral por debajo;
- orientacion razonable o moderadamente desviada;
- centro de prediccion proximo al objeto.

### E3 - Tamano/apertura no plausible

Se asigna E3 cuando la posicion general puede parecer razonable, pero la caja predicha tiene dimensiones claramente inconsistentes con la apertura o extension del objeto.

Indicadores usados:

- `area_ratio_pred_gt` muy alejado de 1;
- la prediccion queda demasiado pequena o demasiado grande;
- el error principal no es de fondo sino de escala/apertura.

### E4 - Confusion por fondo u oclusion

Se asigna E4 cuando la prediccion cae fuera de la region operativa del objeto, normalmente sobre fondo o en una zona desplazada que no sirve como agarre plausible.

Indicadores usados:

- IoU = 0 o practicamente nulo;
- centro de prediccion muy alejado del GT util;
- interpretacion visual de confusion con fondo o region no funcional.

## Aplicacion a los casos seleccionados

### Caso E1 seleccionado

- Imagen: `data/raw/cornell/02/pcd0287r.png`
- Experimento: `EXP1_SIMPLE_RGB`
- Valores observados:
  - IoU = 0.80
  - dtheta = 52.1 deg
- Clasificacion:
  - E1
- Motivo:
  - el solape es alto, asi que el fallo no esta en la localizacion gruesa;
  - el incumplimiento viene del angulo.

### Caso E2 seleccionado

- Imagen: `data/raw/cornell/02/pcd0268r.png`
- Experimento: `EXP1_SIMPLE_RGB`
- Valores observados:
  - IoU = 0.23
  - dtheta = 30.9 deg
- Clasificacion:
  - E2
- Motivo:
  - la prediccion queda en una zona interpretable del objeto, pero el solape es insuficiente;
  - aunque el error angular esta cerca del umbral, el aspecto dominante en la lectura visual es el bajo solape.

### Caso E3 seleccionado

- Imagen: `data/raw/cornell/06/pcd0633r.png`
- Experimento: `EXP3_RESNET18_RGB_AUGMENT`
- Valores observados:
  - IoU = 0.23
  - dtheta = 28.1 deg
  - `area_ratio_pred_gt = 0.23`
- Clasificacion:
  - E3
- Motivo:
  - la escala predicha es demasiado pequena respecto al GT util;
  - el fallo dominante es de apertura/tamano, no de fondo.

### Caso E4 seleccionado

- Imagen: `data/raw/cornell/02/pcd0250r.png`
- Experimento: `EXP1_SIMPLE_RGB`
- Valores observados:
  - IoU = 0.00
  - dtheta = 26.0 deg
  - `center_distance_norm = 3.24`
- Clasificacion:
  - E4
- Motivo:
  - la prediccion cae claramente fuera del objeto, sobre fondo;
  - la lectura visual es de confusion espacial mas que de orientacion.

## Casos frontera

- El caso seleccionado como E2 tiene `dtheta = 30.9 deg`, muy cerca del umbral angular.
- Se mantiene como E2 porque:
  - visualmente el fallo dominante es el bajo solapamiento;
  - para E1 ya se ha escogido un ejemplo mucho mas puro y pedagogico.
