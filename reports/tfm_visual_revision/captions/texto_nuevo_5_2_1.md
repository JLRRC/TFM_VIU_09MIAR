# Borrador de texto para la seccion 5.2.1

En los aciertos observados, las predicciones tienden a concentrarse sobre regiones funcionales del objeto y a mantener una orientacion compatible con el eje principal del agarre. La nueva figura de aciertos muestra ejemplos representativos en los que la superposicion entre el GT y la prediccion resulta coherente tanto en posicion como en orientacion, cumpliendo simultaneamente los umbrales de IoU y discrepancia angular del criterio Cornell.

Mas concretamente, estos aciertos suelen aparecer cuando el objeto presenta una silueta relativamente bien delimitada y la red localiza una zona de agarre estable, con aperturas y centros plausibles. Aunque la coincidencia no es perfecta en todos los casos, los ejemplos incluidos permiten apreciar que el modelo conserva la geometria esencial del agarre correcto y evita desviaciones que invalidarian el criterio de exito.

Por ello, la figura de aciertos debe interpretarse como apoyo visual a los patrones positivos detectados en validacion: buena alineacion general del rectangulo predicho con el objeto, solape suficiente con al menos una anotacion GT valida y errores angulares contenidos dentro del margen aceptado por el protocolo Cornell. Esta separacion respecto a la figura de fallos facilita distinguir con mayor claridad los aciertos geometricamente plausibles de los errores tipificados en E1-E4.
