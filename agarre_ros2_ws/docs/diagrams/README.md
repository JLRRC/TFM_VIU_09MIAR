# Diagramas formales

PUML (PlantUML) source — convertibles a SVG/PNG con `plantuml`:

```bash
# Local
plantuml architecture.puml      # genera architecture.png
plantuml -tsvg architecture.puml  # genera architecture.svg

# Online (sin install)
# https://www.plantuml.com/plantuml/uml/<encoded>
```

## Diagramas

- [architecture.puml](architecture.puml): componentes del sistema +
  flujo canónico vs fallback. Reemplaza/complementa el Mermaid de
  `architecture_post_legacy.md` para la defensa académica (PNG/SVG).
- [fsm_pick_phases.puml](fsm_pick_phases.puml): FSM detallado de las
  9 fases del orchestrator con gates, errores tipados y refs a fixes
  conocidos (F1.8 top-down, F1.7 path_tol, B-iter9 retry).

## Por qué PUML además de Mermaid

- Mermaid renderiza en GitHub directamente (cómodo para README).
- PUML genera SVG/PNG embebibles en LaTeX para defensa formal del TFM.
- Audit-v4 close añade ambos: Mermaid para web, PUML para paper.
