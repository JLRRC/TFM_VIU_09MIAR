# PLAN AUTONOMO PRIORIZADO MASTER

Fecha: 2026-03-23

## Backlog real priorizado
1. Cerrar arranque limpio y orquestacion estable.
2. Eliminar bloqueos falsos de release/freshness en rutas remotas.
3. Cerrar APPROACH/PRE_GRASP/GRASP_DOWN en pick_object.
4. Extender a LIFT/TRANSPORT/RELEASE.
5. Medir repetibilidad por mini bateria.

## Fases y criterio de cierre

## Fase 1 - Arranque y orquestacion fiable
Criterio de cierre:
- system_state=READY
- servicios panel activos
- select/pick/tfm services responden
Estado: CERRADA

## Fase 2 - Tramo corto canonico funcional
Criterio de cierre:
- APPROACH + PRE_GRASP + GRASP_DOWN con resultado terminal util
Estado: BLOQUEADA
Bloqueo: APPROACH queda esperando result topic pese a bridge vivo

## Fase 3 - Ciclo completo funcional
Criterio de cierre:
- LIFT + TRANSPORT + RELEASE sin regresion de fase 2
Estado: BLOQUEADA por dependencia de Fase 2

## Fase 4 - Repetibilidad y evidencia TFM
Criterio de cierre:
- mini bateria de corridas
- clasificacion por rutas
- evidencia reproducible
Estado: VALIDADA CON LIMITACIONES (parcial)

## Siguiente frente exacto
Frente unico y dominante:
- instrumentar y acotar el bucle request->result de pick_object APPROACH para forzar resultado util o fallo rapido diagnostico (evitar waits de 420s), con foco en ur5_moveit_bridge + publish contract en /desired_grasp/result.
