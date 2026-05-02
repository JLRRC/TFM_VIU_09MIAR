# Resumen de congreso - CII 2026 UNET

## Datos para formulario

- Congreso objetivo: II Congreso Internacional de Investigacion 2026, UNET.
- Categoria recomendada: resultados de investigacion concluida.
- Modalidad: ponencia virtual asincrona en video si el resumen/articulo es aceptado.
- Area tematica final: Ciencias Exactas.
- Autor 1: Jesús Lozano Rodríguez (`jesus.lozano.rodriguez@gmail.com`), Universidad Internacional de Valencia (VIU), España.
- Autor 2 / tutor: Dr. Jesús Marcey García (`jmgarcia@unet.edu.ve`), Universidad Nacional Experimental del Táchira (UNET), Venezuela.
- CRediT propuesto autor 1: conceptualizacion, metodologia, software, validacion, analisis formal, investigacion, visualizacion y redaccion del borrador original.
- CRediT propuesto autor 2: supervision, revision y edicion, validacion academica y orientacion metodologica.
- Advertencia: la convocatoria indica doble ciego y exige PDF anonimo. Los autores deben ir en el formulario salvo instruccion contraria del congreso.

## Titulo propuesto

HACIA UN GRASPING 2D/2.5D REPRODUCIBLE: COMPARACIÓN DE ARQUITECTURAS CNN EN ENTORNOS NO ESTRUCTURADOS

## Titulo corto

GRASPING 2D REPRODUCIBLE

## Palabras clave

grasping 2D/2.5D; redes convolucionales; RGB-D; reproducibilidad; vision artificial

## Resumen anonimo para PDF

En escenas no estructuradas, un agarre visualmente plausible puede fallar si la geometría estimada no es suficientemente precisa, comparable y transferible a una acción robótica. El grasping 2D/2.5D aborda este reto transformando información RGB o RGB-D en rectángulos de agarre evaluables; sin embargo, la comparación entre modelos suele verse condicionada por decisiones experimentales poco visibles, como la arquitectura, la modalidad de entrada, la partición de datos, el aumento de datos, las semillas o la métrica de selección. En este contexto, la problemática abordada es la necesidad de un protocolo reproducible que permita comparar arquitecturas CNN sin confundir mejoras reales con efectos derivados del procedimiento experimental. El objetivo del estudio es analizar, bajo condiciones controladas, el rendimiento relativo de una CNN ligera y de una arquitectura residual basada en ResNet-18 para la regresión de rectángulos de agarre siguiendo la representación tipo Cornell en entornos no estructurados. Metodológicamente, los experimentos se definieron mediante archivos YAML, se empleó una partición object-wise del Cornell Grasping Dataset, se entrenaron tres semillas por configuración con PyTorch y se evaluó cada modelo mediante éxito de agarre, IoU y error angular. La comparación incluyó entradas RGB y RGB-D, configuraciones con y sin data augmentation, y medidas complementarias de eficiencia, manteniendo trazabilidad completa de los artefactos experimentales. Los resultados muestran que ResNet18Grasp en RGB con data augmentationobtuvo el mejor rendimiento medio, con un val_success de 0,6801 ± 0,0245, un IoU de 0,4010 ± 0,0046 y un error angular de 10,6071 ± 1,1800 grados. La discusión indica que la arquitectura residual mejora de forma clara a la CNN ligera, mientras que la incorporación de profundidad no garantiza una ventaja uniforme bajo todas las configuraciones. Como cierre aplicado del flujo de trabajo propuesto, el modelo seleccionado se integró como módulo de percepción en un entorno robótico simulado con ROS 2 y Gazebo, aportando evidencia de integración funcional sin convertir este componente en el eje central del artículo. En conclusión, el trabajo aporta una comparación reproducible, controlada y verificable de arquitecturas CNN para grasping 2D/2.5D, con valor metodológico para futuros estudios sobre generalización, métricas orientadas y despliegue en escenarios robóticos.

## Estado de entrega

- Version entregada al tutor: 2026-05-02.
- Conteo comunicado en la version enviada: 308 palabras.
- Estado editorial: resumen entregado y bloqueado. No modificar el texto salvo indicacion expresa.

## Pendiente administrativo

- Mantener esta version como base de envio. Solo aplicar cambios si el tutor o el formulario lo indican expresamente.
- Ajustar el documento final a Times New Roman 12, interlineado 1.5, margenes 3 cm y una sola pagina/carta en PDF.
- Eliminar cualquier identificador de autor del PDF antes del envio oficial si se mantiene la regla de anonimato de la convocatoria.
- Afiliaciones confirmadas para el formulario: Jesús Lozano Rodríguez, Universidad Internacional de Valencia (VIU), España; Dr. Jesús Marcey García, Universidad Nacional Experimental del Táchira (UNET), Venezuela.
