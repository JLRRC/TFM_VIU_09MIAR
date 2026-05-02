# Plan editorial del articulo

## Objetivo

Convertir los resultados experimentales disponibles en un articulo cientifico con gancho de congreso. El articulo debe vender una idea clara: un estudio reproducible y controlado de arquitecturas CNN para grasping 2D/2.5D en entornos no estructurados, con cierre aplicado mediante integracion funcional.

## Mensaje central

Este trabajo propone una comparacion reproducible y controlada de arquitecturas CNN para deteccion de agarres 2D/2.5D en entornos no estructurados, analizando el efecto de la arquitectura, la modalidad de entrada y el aumento de datos, y cerrando con una validacion funcional de aplicabilidad.

## Prioridad inmediata: resumen CII 2026 UNET

- Preparar PDF anonimo del resumen: 300-500 palabras, espanol, carta, Times New Roman 12, interlineado 1.5, margenes 3 cm, un solo parrafo justificado.
- Usar el titulo rector; tiene 13 palabras y cumple el limite de 20 palabras.
- Usar para formulario: Jesús Lozano Rodríguez (`jesus.lozano.rodriguez@gmail.com`), Universidad Internacional de Valencia (VIU), España; Dr. Jesús Marcey García (`jmgarcia@unet.edu.ve`), Universidad Nacional Experimental del Táchira (UNET), Venezuela.
- Afiliaciones confirmadas para el formulario: Jesús Lozano Rodríguez, Universidad Internacional de Valencia (VIU), España; Dr. Jesús Marcey García, Universidad Nacional Experimental del Táchira (UNET), Venezuela.
- Mantener el PDF sin identificadores personales si aplica la revision doble ciego.
- Fecha limite localizada en la web del congreso: viernes 8 de mayo de 2026.

## CRediT propuesto

- Jesús Lozano Rodríguez: conceptualizacion, metodologia, software, validacion, analisis formal, investigacion, visualizacion y redaccion del borrador original.
- Dr. Jesús Marcey García: supervision, revision y edicion, validacion academica y orientacion metodologica.

## Fase 1. Framing

- Abrir con el problema: pasar de imagenes RGB/RGB-D a agarres geometricamente utiles en escenas no estructuradas.
- Presentar la reproducibilidad como necesidad metodologica, no como detalle administrativo.
- Evitar prometer estado del arte, 6-DoF o validacion fisica sistematica.
- Situar la integracion funcional como cierre de aplicabilidad, no como foco principal.

## Fase 2. Curacion de resultados

- Usar `EXP1..EXP4` como bloque principal del articulo.
- Incluir `EXP1.1/EXP1.2` dentro del articulo como bloque complementario, claramente separado de `EXP1..EXP4`.
- Reportar `success`, IoU, error angular, `val_loss`, latencia y tamano de modelo sin exagerar su alcance.
- Separar resultados oficiales de variantes metodologicas pendientes como `GraspLoss` e IoU orientada.

## Fase 3. Redaccion

- Estructura recomendada: introduccion fuerte, estado del arte breve, metodologia reproducible, modelos evaluados, protocolo, resultados, discusion, aplicabilidad, limitaciones y conclusiones.
- Eliminar lenguaje de documentacion interna o tono administrativo.
- Priorizar frases con sujeto academico: "este estudio", "el protocolo", "los resultados", "la comparacion".
- Compactar detalles de implementacion en anexos cuando no aporten al argumento.

## Fase 4. Validacion final

- Verificar que cada cifra cuantitativa enlaza con CSV interno.
- Revisar que el resumen, la introduccion y las conclusiones cuenten la misma historia.
- Comprobar que la plataforma de integracion no domine el texto.
- Adaptar referencias bibliograficas formales al estilo final del venue.
- Ajustar figuras y tablas al formato del venue.
