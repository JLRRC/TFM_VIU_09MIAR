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

En escenas no estructuradas, un agarre visualmente plausible puede fallar si la geometria estimada no es precisa, comparable y transferible a una accion. El grasping 2D/2.5D aborda este reto transformando informacion RGB o RGB-D en rectangulos de agarre evaluables, pero la comparacion entre modelos suele quedar condicionada por decisiones poco visibles: arquitectura, modalidad de entrada, particion de datos, aumento de datos, semillas y metrica de seleccion. La problematica abordada es la necesidad de un protocolo reproducible que permita comparar arquitecturas CNN sin confundir mejoras reales con efectos del procedimiento experimental. El objetivo del estudio es analizar, bajo condiciones controladas, el rendimiento relativo de una CNN ligera y una arquitectura residual basada en ResNet-18 para regresion de rectangulos de agarre en la representacion tipo Cornell. Metodologicamente, se definieron experimentos mediante archivos YAML, se uso particion object-wise del dataset Cornell, se entrenaron tres semillas por configuracion con PyTorch y se evaluo cada modelo mediante exito de agarre, IoU, error angular y perdida de validacion. La comparacion considera entradas RGB y RGB-D, configuraciones con y sin augmentation y evidencias complementarias de eficiencia, manteniendo trazabilidad completa de los artefactos. Los resultados oficiales muestran que `ResNet18Grasp` en RGB con augmentation obtuvo el mejor rendimiento medio, con `val_success` de 0,6801 +/- 0,0245, IoU de 0,4010 +/- 0,0046 y error angular de 10,6071 +/- 1,1800 grados. La discusion indica que la arquitectura residual mejora de forma clara a la CNN ligera, mientras que la incorporacion de profundidad no garantiza una ventaja uniforme bajo todas las configuraciones. Como cierre aplicado, el modelo seleccionado se conserva como base para inferencia funcional, aportando evidencia de aplicabilidad sin convertir la integracion en el eje del articulo. En conclusion, el trabajo aporta una comparacion reproducible, controlada y verificable de arquitecturas CNN para grasping 2D/2.5D, con valor metodologico para futuros estudios sobre generalizacion, metricas orientadas y despliegue en escenarios roboticos.

## Pendiente antes de enviar

- Ajustar el resumen a Times New Roman 12, interlineado 1.5, margenes 3 cm y una sola pagina/carta en PDF.
- Comprobar conteo final de palabras tras pasar a Word/LibreOffice. Objetivo: 300-500 palabras.
- Eliminar cualquier identificador de autor del PDF si se mantiene la regla de anonimato de la convocatoria.
- Afiliaciones confirmadas para el formulario: Jesús Lozano Rodríguez, Universidad Internacional de Valencia (VIU), España; Dr. Jesús Marcey García, Universidad Nacional Experimental del Táchira (UNET), Venezuela.
