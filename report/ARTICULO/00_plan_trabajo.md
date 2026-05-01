# Plan editorial del articulo

## Prioridad inmediata: resumen CII 2026 UNET

- Preparar PDF anonimo del resumen: 300-500 palabras, espanol, carta, Times New Roman 12, interlineado 1.5, margenes 3 cm, un solo parrafo justificado.
- Usar el titulo rector de `01_resumen_congreso.md`; cumple el limite de 20 palabras.
- Completar en el formulario los autores y la declaracion CRediT.
- Confirmar nombre completo, afiliacion y correo institucional del segundo autor.
- Fecha limite localizada en la web del congreso: viernes 8 de mayo de 2026.

## Fase 1. Delimitacion del mensaje

- Mensaje recomendado: comparacion reproducible de arquitecturas CNN para deteccion de agarres 2D/2.5D bajo un protocolo comun, con enfasis en trazabilidad experimental.
- Resultado central: `ResNet18Grasp` en RGB con augmentation (`EXP3`) obtiene el mejor `val_success` medio entre los experimentos oficiales.
- Aportacion metodologica: configuraciones versionadas, particion object-wise, tres semillas, metricas por epoca, mejor epoca por seed y agregacion verificable.
- Evitar prometer: superioridad frente al estado del arte, generalizacion universal, validacion fisica o conclusion fuerte sobre RGB-D mas alla de este protocolo.

## Fase 2. Curacion de resultados

- Usar como nucleo experimental `EXP1..EXP4`.
- Mantener `EXP1.1/EXP1.2` como evidencia auxiliar de implementacion, no como reemplazo del bloque oficial.
- Declarar explicitamente que la variante con perdida angular e IoU orientada es una extension metodologica pendiente de ejecucion consolidada.
- Separar las cifras del articulo principal de tablas generadas con criterios de desviacion diferentes.

## Fase 3. Redaccion

- Mantener tono de paper: problema, metodo, resultados, discusion, limitaciones y conclusiones.
- Reducir referencias a integracion robotica a contexto aplicado o trabajo futuro.
- Concentrar el estado del arte en Cornell, CNN/ResNet, RGB-D, GGCNN/Jacquard/GraspNet y reproducibilidad experimental.
- Convertir resultados a una tabla principal y 2-4 figuras.

## Fase 4. Validacion antes de enviar

- Comprobar que cada afirmacion cuantitativa enlaza con CSV interno.
- Revisar que no queden formulaciones internas o de desarrollo de software como eje discursivo.
- Confirmar autores, afiliaciones, anonimato y formato.
- Exportar figuras a PDF/SVG si la plantilla lo exige.
