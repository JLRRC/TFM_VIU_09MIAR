# Aportaciones clave

## Que aporta realmente el articulo

- Una comparacion reproducible y controlada de arquitecturas CNN para grasping 2D/2.5D.
- Un protocolo experimental con particion object-wise, configuraciones versionadas, tres semillas por experimento y metricas trazables.
- Una lectura comparativa del efecto de arquitectura, modalidad RGB/RGB-D y augmentation.
- Una evaluacion conjunta de success, IoU, error angular, perdida de validacion, eficiencia y tamano de modelo.
- Una seleccion razonada de la configuracion con mejor `val_success` y cierre de aplicabilidad mediante inferencia funcional.

## Por que es defendible y atractivo

El articulo no depende de prometer una arquitectura nueva. Su valor esta en comparar con rigor un problema practico: que se gana al pasar de una CNN ligera a una arquitectura residual, que aporta realmente RGB-D bajo este protocolo y hasta que punto augmentation puede ser mas determinante que la modalidad de entrada. Ese framing es competitivo para congreso porque conecta reproducibilidad, aprendizaje profundo aplicado y grasping 2D/2.5D sin inflar la aportacion.

## Angulo correcto

El angulo correcto es: "un estudio reproducible sobre decisiones experimentales que cambian el rendimiento en grasping 2D/2.5D". La integracion funcional aparece como evidencia de aplicabilidad final, pero el argumento principal esta en la comparacion controlada y en la interpretacion de resultados.

## Que no se debe vender

- No venderlo como nuevo estado del arte.
- No afirmar generalizacion universal fuera de Cornell.
- No presentar validacion fisica si no existe evidencia sistematica.
- No convertir la plataforma de integracion en el centro del articulo.
- No mezclar resultados oficiales con variantes metodologicas pendientes.
- No ocultar que tres semillas limitan la inferencia estadistica.

## Mensaje para congreso

El articulo puede interesar porque ofrece una historia clara: grasping visual en escenarios no estructurados, comparacion de modelos CNN bajo protocolo exigente, resultados cuantitativos interpretables y cierre funcional. Es una contribucion metodologica seria, compacta y verificable.
