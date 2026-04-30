<!-- generado-articulo-tfm -->

# Plan de trabajo para convertir el TFM en articulo

## Prioridad inmediata: resumen CII 2026 UNET

- Preparar PDF anonimo del resumen segun convocatoria: 300-500 palabras, espanol, carta, Times New Roman 12, interlineado 1.5, margenes 3 cm, un solo parrafo justificado.
- Usar el titulo propuesto en `01_resumen_congreso.md` o una variante de maximo 20 palabras.
- Completar en el formulario los autores: Jesus Lozano Rodriguez como primer autor y tutor como segundo autor.
- Confirmar datos del tutor y declaracion CRediT antes del envio.
- Fecha limite localizada en la web del congreso: viernes 8 de mayo de 2026.

## Fase 1. Delimitacion del mensaje

- Mensaje recomendado: comparacion reproducible de arquitecturas CNN para regresion de agarres 2D/2.5D sobre Cornell, con integracion funcional en un pipeline ROS 2/Gazebo/UR5.
- Resultado central: `ResNet18Grasp` en RGB con augmentation (`EXP3`) obtiene el mejor `val_success` medio entre los experimentos oficiales.
- Evitar prometer: generalizacion real en robot fisico, evaluacion 6-DoF, superioridad universal de RGB-D, o validacion estadistica amplia mas alla de las tres seeds disponibles.

## Fase 2. Curacion de resultados

- Usar como nucleo experimental oficial `EXP1..EXP4`.
- Mantener `EXP1.1/EXP1.2` como evidencia auxiliar de alineacion con el diseno teorico, no como sustituto de las tablas oficiales.
- Revisar si la metrica oficial axis-aligned debe declararse explicitamente frente a la formulacion orientada posterior.

## Fase 3. Redaccion

- Transformar el borrador en un articulo de 6-8 paginas o en resumen extendido segun venue.
- Reducir estado del arte a trabajos estrictamente necesarios: Cornell, Jacquard/GGCNN, Dex-Net/GraspNet si aplica, y CNN/ResNet para grasping 2D.
- Convertir resultados a 2 tablas principales y 3-4 figuras.

## Fase 4. Validacion antes de enviar

- Comprobar que cada afirmacion cuantitativa enlaza con CSV interno.
- Confirmar autores, afiliaciones, anonimato y formato.
- Exportar figuras a PDF/SVG si la plantilla lo exige.
