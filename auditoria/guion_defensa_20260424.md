# Guion corto para defensa tecnica

Duracion objetivo: menos de 2 minutos.

## Version oral breve

El launcher canónico del proyecto es `./lanzar_panelc2.sh`, ejecutado desde la raiz del workspace. Debe usarse ese launcher porque es la entrada operativa que deja el overlay correcto del workspace y evita desalineaciones entre `src`, `build` e `install`; si se arranca por rutas manuales o wrappers internos, la verdad runtime puede no coincidir con el código fuente que creemos estar validando.

El flujo DIRECTO no está roto globalmente, pero tampoco está completamente cerrado. La parte delantera del ciclo, incluyendo approach, grasp, cierre, attach y varios slices de transporte, sí está validada en código, tests y build. En concreto, el slice de `RECOVER_2` ya tiene parche local, cobertura unitaria y build correcto. Lo que sigue pendiente es la validación runtime canónica del tramo final de transporte a cesta, porque la corrida actual vuelve a caer antes en `CESTA_STAGE_1_RECOVER_1`, y por eso `RECOVER_2` todavía no puede declararse cerrado de extremo a extremo.

La diferencia importante es esta: hoy tenemos una integración demostrativa sólida en varias fases críticas, pero no un sistema completamente cerrado para afirmar sin reservas el ciclo DIRECTO completo hasta cesta y retorno final bajo el runner canónico. Y en el bloque de visión hay otra separación que conviene explicar bien: los resultados oficiales del TFM siguen siendo `EXP1..EXP4`, mientras que la alineación metodológica posterior añade evaluación orientada, `GraspLoss` y experimentos auxiliares sin reescribir ni sustituir esos resultados oficiales.

## Version en puntos de apoyo

- Launcher canónico: `./lanzar_panelc2.sh`.
- Motivo: fija el overlay correcto y reduce contaminación entre `src`, `build` e `install`.
- Estado DIRECTO: parcialmente validado; parte delantera y varios slices de transporte están controlados.
- Validado: tests del slice de transporte, build de `ur5_qt_panel`, blindajes de panel y parche local de `RECOVER_2`.
- Pendiente: `CESTA_STAGE_1_RECOVER_1` sigue bloqueando la corrida canónica; por eso `RECOVER_2` aún no tiene cierre runtime extremo a extremo.
- Integración demostrativa vs cierre total: ya hay integración útil y defendible, pero no cierre completo del trayecto cesta más retorno final bajo validación canónica actual.
- CNN: `EXP1..EXP4` son resultados oficiales; la alineación metodológica posterior es una mejora del repo, no una reescritura de la memoria.