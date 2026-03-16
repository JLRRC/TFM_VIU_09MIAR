#!/usr/bin/env python3

from __future__ import annotations

import csv
import subprocess
import shutil
import unicodedata
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


@dataclass(frozen=True)
class Item:
    kind: str
    number: str
    title: str
    page: str | None = None
    assets: tuple[str, ...] = ()
    references: tuple[str, ...] = ()
    notes: str | None = None
    render_pdf_page: bool = False


@dataclass(frozen=True)
class Chapter:
    number: str
    slug: str
    title: str
    page_range: str | None = None
    illustrations: tuple[Item, ...] = ()
    tables: tuple[Item, ...] = ()
    artifacts: tuple[Item, ...] = ()


def slugify(text: str) -> str:
    normalized = unicodedata.normalize("NFKD", text)
    ascii_text = normalized.encode("ascii", "ignore").decode("ascii")
    cleaned = []
    for char in ascii_text.lower():
        if char.isalnum():
            cleaned.append(char)
        else:
            cleaned.append("_")
    slug = "".join(cleaned)
    while "__" in slug:
        slug = slug.replace("__", "_")
    return slug.strip("_")[:80] or "item"


def title_case_kind(kind: str) -> str:
    return {
        "illustration": "Ilustracion",
        "table": "Tabla",
        "artifact": "Artefacto",
    }[kind]


def chapter_dir_name(chapter: Chapter) -> str:
    return f"{chapter.number}_{chapter.slug}"


def item_dir(base: Path, kind: str) -> Path:
    folder = {
        "illustration": "ilustraciones",
        "table": "tablas",
        "artifact": "artefactos",
    }[kind]
    return base / folder


def stem_for(item: Item) -> str:
    return f"{title_case_kind(item.kind)}_{item.number}_{slugify(item.title)}"


def manifest_text(chapter: Chapter, item: Item, copied_assets: Iterable[Path]) -> str:
    copied_assets = list(copied_assets)
    lines: list[str] = []
    lines.append(f"# {title_case_kind(item.kind)} {item.number}")
    lines.append("")
    lines.append(f"- Titulo: {item.title}")
    lines.append(f"- Capitulo: {chapter.number}. {chapter.title}")
    if chapter.page_range:
        lines.append(f"- Rango del capitulo en el PDF: {chapter.page_range}")
    if item.page:
        lines.append(f"- Pagina en el PDF: {item.page}")
    lines.append(f"- Tipo: {title_case_kind(item.kind)}")
    if copied_assets:
        lines.append("- Estado: artefacto materializado dentro de reports/capitulos")
    else:
        lines.append("- Estado: referencia documental sin fichero independiente localizado fuera del PDF")
    if item.notes:
        lines.append(f"- Nota: {item.notes}")
    if copied_assets:
        lines.append("- Ficheros asociados:")
        for asset in copied_assets:
            lines.append(f"  - {asset.name}")
    if item.references or item.assets:
        lines.append("- Fuentes de trazabilidad:")
        for ref in item.assets:
            lines.append(f"  - {ref}")
        for ref in item.references:
            lines.append(f"  - {ref}")
    lines.append("")
    return "\n".join(lines)


def copy_assets(root: Path, destination_dir: Path, item: Item) -> list[Path]:
    copied: list[Path] = []
    stem = stem_for(item)
    for asset_rel in item.assets:
        src = root / asset_rel
        if not src.exists():
            continue
        dst = destination_dir / f"{stem}{src.suffix.lower()}"
        shutil.copy2(src, dst)
        copied.append(dst)
    return copied


def render_pdf_page(root: Path, destination_dir: Path, item: Item, pdf_rel: str) -> list[Path]:
    if not item.render_pdf_page or not item.page:
        return []
    pdf_path = root / pdf_rel
    if not pdf_path.exists():
        return []

    stem = stem_for(item)
    out_prefix = destination_dir / f"{stem}_pagina"
    expected = destination_dir / f"{stem}_pagina-{item.page}.png"
    final = destination_dir / f"{stem}_pagina.png"

    try:
        subprocess.run(
            [
                "pdftoppm",
                "-png",
                "-f",
                item.page,
                "-l",
                item.page,
                str(pdf_path),
                str(out_prefix),
            ],
            check=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except (FileNotFoundError, subprocess.CalledProcessError):
        return []

    if expected.exists():
        expected.replace(final)
        return [final]
    if final.exists():
        return [final]
    return []


def write_manifest(path: Path, chapter: Chapter, item: Item, copied_assets: Iterable[Path]) -> None:
    path.write_text(manifest_text(chapter, item, copied_assets), encoding="utf-8")


def build_filtered_exp4_table(root: Path, destination: Path) -> None:
    summary_path = root / "reports/tables/summary_results.csv"
    if not summary_path.exists():
        return
    with summary_path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        rows = [row for row in reader if row.get("experiment") == "EXP4_RESNET18_RGBD"]
    if not rows:
        return
    with destination.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def chapter_readme(chapter: Chapter, chapter_path: Path) -> str:
    lines = [f"# Capitulo {chapter.number}. {chapter.title}", ""]
    if chapter.page_range:
        lines.append(f"- Paginas del PDF: {chapter.page_range}")
    lines.append(f"- Ruta: {chapter_path.relative_to(chapter_path.parents[2])}")
    lines.append("")
    lines.append("## Contenido")
    lines.append("")
    for label, items in [
        ("Ilustraciones", chapter.illustrations),
        ("Tablas", chapter.tables),
        ("Artefactos", chapter.artifacts),
    ]:
        lines.append(f"### {label}")
        if not items:
            lines.append("")
            lines.append("- Sin elementos catalogados para este apartado.")
            continue
        lines.append("")
        for item in items:
            lines.append(f"- {title_case_kind(item.kind)} {item.number}: {item.title}")
    lines.append("")
    return "\n".join(lines)


def reports_readme(chapters: Iterable[Chapter]) -> str:
    lines = [
        "# Reports del TFM",
        "",
        "Esta carpeta combina dos vistas complementarias:",
        "",
        "- `capitulos/`: vista editorial alineada con la numeracion real del TFM.",
        "- resto de subcarpetas (`figures/`, `tables/`, `tfm_ros_gazebo_results/`, etc.): vista tecnica y de artefactos fuente regenerables.",
        "",
        "La carpeta `capitulos/` es la referencia principal para localizar ilustraciones, tablas y evidencias tal y como aparecen numeradas en la memoria.",
        "Los ficheros `CORRESPONDENCIA_PDF_ARTEFACTOS_TFM.md` y `CORRESPONDENCIA_PDF_ARTEFACTOS_TFM.csv` actuan como indice maestro PDF -> artefacto real.",
        "",
        "## Capitulos disponibles",
        "",
    ]
    for chapter in chapters:
        page_info = f" (pp. {chapter.page_range})" if chapter.page_range else ""
        lines.append(f"- `{chapter.number}_{chapter.slug}/`: {chapter.title}{page_info}")
    lines.append("")
    return "\n".join(lines)


def inventory_text(chapters: Iterable[Chapter]) -> str:
    lines = [
        "# Inventario de artefactos del TFM",
        "",
        "Inventario alineado con la numeracion del documento `TFM_Jesus_Lozano_V10.pdf`.",
        "La tabla maestra de correspondencia adicional queda en `reports/CORRESPONDENCIA_PDF_ARTEFACTOS_TFM.md` y `reports/CORRESPONDENCIA_PDF_ARTEFACTOS_TFM.csv`.",
        "El PDF maestro del TFM se conserva en `reports/TFM_Jesus_Lozano_V10.pdf`.",
        "",
    ]
    for chapter in chapters:
        lines.append(f"## Capitulo {chapter.number}. {chapter.title}")
        lines.append("")
        if chapter.page_range:
            lines.append(f"- Paginas: {chapter.page_range}")
            lines.append("")
        for label, items in [
            ("Ilustraciones", chapter.illustrations),
            ("Tablas", chapter.tables),
            ("Artefactos", chapter.artifacts),
        ]:
            lines.append(f"### {label}")
            lines.append("")
            if not items:
                lines.append("- Sin elementos catalogados.")
                lines.append("")
                continue
            for item in items:
                page = f" (p. {item.page})" if item.page else ""
                lines.append(f"- {title_case_kind(item.kind)} {item.number}: {item.title}{page}")
            lines.append("")
    return "\n".join(lines)


def mapping_rows(chapters: Iterable[Chapter]) -> list[dict[str, str]]:
    rows: list[dict[str, str]] = []
    for chapter in chapters:
        for group_name, items in (("ilustracion", chapter.illustrations), ("tabla", chapter.tables), ("artefacto", chapter.artifacts)):
            for item in items:
                chapter_folder = f"reports/capitulos/{chapter_dir_name(chapter)}"
                item_folder = {
                    "illustration": "ilustraciones",
                    "table": "tablas",
                    "artifact": "artefactos",
                }[item.kind]
                item_stem = stem_for(item)
                manifest = f"{chapter_folder}/{item_folder}/{item_stem}.md"
                assets = "; ".join(item.assets) if item.assets else ""
                notes = item.notes or ""
                rows.append(
                    {
                        "chapter": chapter.number,
                        "chapter_title": chapter.title,
                        "type": group_name,
                        "number": item.number,
                        "title": item.title,
                        "pdf_page": item.page or "",
                        "chapter_folder": chapter_folder,
                        "manifest": manifest,
                        "source_assets": assets,
                        "notes": notes,
                    }
                )
    return rows


def write_master_mapping(reports_dir: Path, chapters: Iterable[Chapter]) -> None:
    rows = mapping_rows(chapters)
    csv_path = reports_dir / "CORRESPONDENCIA_PDF_ARTEFACTOS_TFM.csv"
    with csv_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=[
                "chapter",
                "chapter_title",
                "type",
                "number",
                "title",
                "pdf_page",
                "chapter_folder",
                "manifest",
                "source_assets",
                "notes",
            ],
        )
        writer.writeheader()
        writer.writerows(rows)

    md_lines = [
        "# Correspondencia PDF -> Artefactos del TFM",
        "",
        "Tabla maestra de correspondencia entre la numeracion del PDF y los artefactos reales organizados en `reports/capitulos/`.",
        "",
        "| Cap. | Tipo | Numero | Titulo | Pag. PDF | Manifest | Fuente real |",
        "|---|---|---|---|---|---|---|",
    ]
    for row in rows:
        source_assets = row["source_assets"] if row["source_assets"] else "Referencia documental"
        md_lines.append(
            f"| {row['chapter']} | {row['type']} | {row['number']} | {row['title']} | {row['pdf_page']} | {row['manifest']} | {source_assets} |"
        )
    md_lines.append("")
    (reports_dir / "CORRESPONDENCIA_PDF_ARTEFACTOS_TFM.md").write_text("\n".join(md_lines), encoding="utf-8")


def build_chapters() -> tuple[Chapter, ...]:
    pdf_ref = "reports/TFM_Jesus_Lozano_V10.pdf"
    trace_ref = "agarre_inteligente/docs/TRAZABILIDAD_TFM.md"
    validation_ref = "reports/docs/workspace/VALIDACION_WORKSPACE_2026_03_16.md"
    release_ref = "reports/docs/workspace/RELEASE_v1.0.0.md"

    return (
        Chapter(
            number="01",
            slug="introduccion",
            title="Introduccion",
            page_range="11-13",
            illustrations=(
                Item("illustration", "1-1", "Ejemplos de escenarios no estructurados para detección de poses de agarre", "11", references=(pdf_ref,), render_pdf_page=True),
                Item("illustration", "1-2", "Representación 2D/2.5D del agarre como rectángulo orientado", "12", references=(pdf_ref,), render_pdf_page=True),
                Item(
                    "illustration",
                    "1-3",
                    "Diagrama general del pipeline del trabajo e integración",
                    "13",
                    assets=("reports/figures/diagrama_pipeline.png",),
                    references=(pdf_ref, trace_ref),
                ),
            ),
        ),
        Chapter(
            number="02",
            slug="objetivos_y_alcance",
            title="Objetivos y alcance",
            page_range="14-17",
            illustrations=(
                Item("illustration", "2-1", "Mapa de trazabilidad del pipeline (dataset → modelo → métricas → ROS 2/Gazebo)", "15", references=(pdf_ref, trace_ref), render_pdf_page=True),
            ),
            tables=(
                Item("table", "2-1", "Trazabilidad entre objetivos específicos y evidencias en el documento", "15", references=(pdf_ref, trace_ref), render_pdf_page=True),
                Item("table", "2-2", "Fuera de alcance vs. extensiones futuras", "17", references=(pdf_ref,), render_pdf_page=True),
            ),
        ),
        Chapter(
            number="03",
            slug="estado_del_arte_y_marco_teorico",
            title="Estado del Arte y Marco teorico",
            page_range="18-32",
            illustrations=(
                Item("illustration", "3-1", "Taxonomía del problema de agarre: 2D/2.5D vs 6-DoF", "19", references=(pdf_ref,), render_pdf_page=True),
                Item("illustration", "3-2", "Ejemplo conceptual de mapas densos (calidad, ángulo, apertura) estilo GG-CNN", "20", references=(pdf_ref,), render_pdf_page=True),
                Item("illustration", "3-3", "Ejemplos de degradación por oclusión/fondo: tipología de fallos", "22", references=(pdf_ref,), render_pdf_page=True),
                Item("illustration", "3-4", "Mapa de posicionamiento: estado del arte → decisiones del trabajo", "24", references=(pdf_ref,), render_pdf_page=True),
                Item("illustration", "3-5", "Parámetros del rectángulo de agarre (Cx, Cy, w, h, θ) sobre una imagen", "28", references=(pdf_ref,), render_pdf_page=True),
                Item("illustration", "3-6", "Interpretación geométrica de IoU y Δθ en rectángulos orientados", "32", references=(pdf_ref,), render_pdf_page=True),
            ),
            tables=(
                Item("table", "3-1", "Comparativa sintética de enfoques 2D/2.5D: mapas densos vs regresión paramétrica", "21", references=(pdf_ref,), render_pdf_page=True),
                Item("table", "3-2", "Datasets relevantes: Cornell/Jacquard (2D/2.5D) vs GraspNet/ACRONYM (6-DoF)", "25", references=(pdf_ref,), render_pdf_page=True),
            ),
        ),
        Chapter(
            number="04",
            slug="metodologia",
            title="Metodologia",
            page_range="33-54",
            illustrations=(
                Item("illustration", "4-1", "Vision global del pipeline experimental", "33", assets=("reports/figures/diagrama_pipeline.png",), references=(pdf_ref, trace_ref)),
                Item("illustration", "4-2", "Ejemplo de anotaciones Cornell sobre una imagen", "36", references=(pdf_ref,), render_pdf_page=True),
                Item("illustration", "4-3", "Flujo del pipeline: auditoría → índices limpios → dataset Subset estricto", "38", references=(pdf_ref, trace_ref), render_pdf_page=True),
                Item("illustration", "4-4", "Ejemplo de evaluación tipo Cornell con fallo por incumplimiento de umbrales de IoU y/o Δθ", "47", references=(pdf_ref,), render_pdf_page=True),
                Item("illustration", "4-5", "Galería cualitativa: 4 aciertos + 4 fallos con explicación del tipo de fallo", "48", references=(pdf_ref,), render_pdf_page=True),
                Item("illustration", "4-6", "Arquitectura ROS 2 del entorno simulado (nodos y tópicos)", "50", references=(pdf_ref, validation_ref), render_pdf_page=True),
                Item("illustration", "4-7", "Entorno de emulación ROS 2/Gazebo: ejemplo de consistencia visual (overlay) y consumo de la hipótesis de agarre en la escena table-top", "52", assets=("reports/tfm_ros_gazebo_results/evidence/images/ros_gazebo_imagen_camara.png",), references=(pdf_ref, validation_ref), notes="Se usa la captura de escena simulada como soporte visual más fiel a la descripción editorial de entorno y adquisición."),
            ),
            tables=(
                Item("table", "4-1", "Fases del trabajo, entradas, salidas y evidencias de verificación", "35", references=(pdf_ref, trace_ref), render_pdf_page=True),
                Item("table", "4-2", "Estadísticas del split utilizado en el conjunto experimental final", "37", references=(pdf_ref,), render_pdf_page=True),
                Item("table", "4-3", "Transformaciones de data augmentation y cómo se actualizan las etiquetas Cx, Cy, w, h, θ", "40", references=(pdf_ref,), render_pdf_page=True),
                Item("table", "4-4", "Comparativa estructural de los modelos (A vs B)", "44", references=(pdf_ref,), render_pdf_page=True),
                Item("table", "4-5", "Configuración experimental por experimento", "45", references=(pdf_ref, trace_ref), render_pdf_page=True),
                Item("table", "4-6", "Hiperparámetros de entrenamiento por experimento (según YAML asociado)", "46", references=(pdf_ref, trace_ref), render_pdf_page=True),
                Item("table", "4-7", "Definición de métricas de evaluación (Cornell): IoU, Δθ y grasp success", "47", references=(pdf_ref, trace_ref), render_pdf_page=True),
                Item("table", "4-8", "Protocolo de medición de latencia", "49", references=(pdf_ref, trace_ref), render_pdf_page=True),
                Item("table", "4-9", "Especificaciones de hardware utilizadas", "53", references=(pdf_ref, validation_ref), render_pdf_page=True),
            ),
            artifacts=(
                Item("artifact", "4-A1", "Diagrama de arquitectura del modelo ResNet18Grasp", "41", assets=("reports/figures/diagrama_arquitectura_resnet18.png",), references=(trace_ref,)),
                Item("artifact", "4-A2", "Diagrama de arquitectura del modelo SimpleGraspCNN", "42", assets=("reports/figures/diagrama_arquitectura_simplecnn.png",), references=(trace_ref,)),
            ),
        ),
        Chapter(
            number="05",
            slug="resultados_y_discusion",
            title="Resultados y discusion",
            page_range="56-77",
            illustrations=(
                Item("illustration", "5-1", "Curvas de perdida (train_loss y val_loss) para EXP1", "57", assets=("reports/tfm_figuras_cap5_1/ilustracion_5_2_curvas_loss_exp1_simple_rgb.png",), references=(pdf_ref, trace_ref), notes="El fichero fuente historico estaba numerado como 5_2; aqui se reordena segun la numeracion real del PDF."),
                Item("illustration", "5-2", "Curvas de perdida (train_loss y val_loss) para EXP2", "57", assets=("reports/tfm_figuras_cap5_1/ilustracion_5_3_curvas_loss_exp2_simple_rgbd.png",), references=(pdf_ref, trace_ref), notes="El fichero fuente historico estaba numerado como 5_3; aqui se reordena segun la numeracion real del PDF."),
                Item("illustration", "5-3", "Curvas de perdida (train_loss y val_loss) para EXP3", "57", assets=("reports/tfm_figuras_cap5_1/ilustracion_5_4_curvas_loss_exp3_resnet18_rgb_augment.png",), references=(pdf_ref, trace_ref), notes="El fichero fuente historico estaba numerado como 5_4; aqui se reordena segun la numeracion real del PDF."),
                Item("illustration", "5-4", "Curvas de perdida (train_loss y val_loss) para EXP4", "57", assets=("reports/tfm_figuras_cap5_1/ilustracion_5_5_curvas_loss_exp4_resnet18_rgbd.png",), references=(pdf_ref, trace_ref), notes="El fichero fuente historico estaba numerado como 5_5; aqui se reordena segun la numeracion real del PDF."),
                Item("illustration", "5-5", "Comparativa de val_success por seed y experimento en best_epoch", "58", assets=("reports/tfm_figuras_cap5_1/ilustracion_5_6_val_success_por_seed_y_experimento.png",), references=(pdf_ref, trace_ref), notes="El fichero fuente historico estaba numerado como 5_6; aqui se reordena segun la numeracion real del PDF."),
                Item("illustration", "5-6", "Comparativa de val_loss por seed y experimento en best_epoch", "59", assets=("reports/tfm_figuras_cap5_1/ilustracion_5_7_val_loss_por_seed_y_experimento.png",), references=(pdf_ref, trace_ref), notes="El fichero fuente historico estaba numerado como 5_7; aqui se reordena segun la numeracion real del PDF."),
                Item("illustration", "5-7", "Evolucion del exito de agarre en validacion por epoca", "59", assets=("reports/figures/val_success_by_epoch.png",), references=(pdf_ref, trace_ref)),
                Item("illustration", "5-8", "Evolucion del IoU medio en validacion por epoca", "60", assets=("reports/figures/val_iou_by_epoch.png",), references=(pdf_ref, trace_ref)),
                Item("illustration", "5-9", "Evolucion del error angular medio en validacion por epoca", "60", assets=("reports/figures/val_angle_deg_by_epoch.png",), references=(pdf_ref, trace_ref)),
                Item("illustration", "5-10", "Exito final de agarre en validacion agregado por experimento", "62", assets=("reports/figures/bar_val_success_final.png",), references=(pdf_ref, trace_ref)),
                Item("illustration", "5-11", "IoU medio final en validacion agregado por experimento", "62", assets=("reports/figures/bar_val_iou_final.png",), references=(pdf_ref, trace_ref)),
                Item("illustration", "5-12", "Error angular medio final en validacion agregado por experimento", "62", assets=("reports/figures/bar_val_angle_final.png",), references=(pdf_ref, trace_ref)),
                Item("illustration", "5-13", "Aciertos representativos en validacion para EXP3_RESNET18_RGB_AUGMENT", "64", assets=("reports/tfm_visual_revision/final/cap5_aciertos_cualitativos_v1.png", "reports/tfm_visual_revision/final/cap5_aciertos_cualitativos_v1.pdf"), references=(pdf_ref,)),
                Item("illustration", "5-14", "Fallos cualitativos representativos en validacion organizados por tipologia", "66", assets=("reports/tfm_visual_revision/final/cap5_fallos_cualitativos_E1_E4_v1.png", "reports/tfm_visual_revision/final/cap5_fallos_cualitativos_E1_E4_v1.pdf"), references=(pdf_ref,)),
                Item("illustration", "5-15", "Caso ilustrativo de objeto pequeno en la escena", "68", assets=("reports/tfm_visual_revision/final/cap5_caso_ilustrativo_v1.png", "reports/tfm_visual_revision/final/cap5_caso_ilustrativo_v1.pdf"), references=(pdf_ref,)),
                Item("illustration", "5-16", "Evidencia funcional del pipeline percepcion-publicacion-consumo en ROS 2", "71", assets=("reports/tfm_ros_gazebo_results/evidence/images/ros_gazebo_frame_panel_infer.png",), references=(pdf_ref, validation_ref, release_ref)),
                Item("illustration", "5-17", "Resultado de inferencia del modelo EXP3_RESNET18_RGB_AUGMENT sobre la imagen simulada", "72", assets=("reports/tfm_ros_gazebo_results/evidence/images/ros_gazebo_overlay_prediccion.png",), references=(pdf_ref, validation_ref)),
                Item("illustration", "5-18", "Evidencia funcional adicional del pipeline percepcion-publicacion-consumo en ROS 2", "73", assets=("reports/tfm_ros_gazebo_results/evidence/images/ros_gazebo_exp3_vs_referencia.png",), references=(pdf_ref, validation_ref), notes="Se prioriza la comparativa visual del caso integrado frente al overlay auxiliar del panel, por ser mas coherente con la narrativa del TFM."),
            ),
            tables=(
                Item("table", "5-1", "Resultados agregados en validacion (best_epoch por ejecucion) bajo split object-wise", "61", assets=("reports/tables/table_validation_aggregated.csv",), references=(pdf_ref, trace_ref)),
                Item("table", "5-2", "Resumen de metricas finales por experimento en validacion", "62", assets=("reports/tables/table_metrics_final.csv",), references=(pdf_ref, trace_ref)),
                Item("table", "5-3", "Medicion de latencia de inferencia por experimento y dispositivo", "69", assets=("reports/tables/table_latency.csv", "reports/bench/latency_results.csv"), references=(pdf_ref, trace_ref)),
                Item("table", "5-4", "Comparativa por modalidad entre SimpleGraspCNN y ResNet18Grasp", "77", assets=("reports/tables/table_ab_comparison_by_modality.csv",), references=(pdf_ref, trace_ref)),
            ),
            artifacts=(
                Item("artifact", "5-A1", "Resumen tecnico para la redaccion de la subseccion ROS 2 + Gazebo", assets=("reports/tfm_ros_gazebo_results/final_pack/resumen_para_redaccion.md",), references=(validation_ref, release_ref)),
                Item("artifact", "5-A2", "Artefacto principal de inferencia y grasp publicado", assets=("reports/panel_audit/artifacts/grasp_last.json",), references=(validation_ref,)),
                Item("artifact", "5-A3", "Resumen tabular completo del bloque experimental", assets=("reports/tables/TABLES_SUMMARY.md",), references=(trace_ref,)),
            ),
        ),
        Chapter(
            number="06",
            slug="conclusiones_y_trabajos_futuros",
            title="Conclusiones y trabajos futuros",
            page_range="78-83",
            tables=(
                Item("table", "6-1", "Síntesis de cumplimiento de objetivos", "79", references=(pdf_ref, validation_ref, trace_ref), render_pdf_page=True),
            ),
        ),
        Chapter(
            number="07",
            slug="referencias_bibliograficas",
            title="Referencias bibliograficas",
            page_range="84-85",
        ),
        Chapter(
            number="08",
            slug="anexos",
            title="Anexos",
            page_range="86-88",
            tables=(
                Item("table", "8-1", "Resultados por semilla y experimento en la mejor epoca de validacion", "86", assets=("reports/tables/results_by_seed.csv",), references=(pdf_ref, trace_ref)),
                Item("table", "8-2", "Resultados de validación del experimento de referencia para la integración ROS 2 (EXP4_RESNET18_RGBD), reportados por semilla en best_epoch", "86", references=(pdf_ref, validation_ref), notes="Se materializa a partir de summary_results.csv filtrando el experimento de referencia citado en el PDF actual."),
                Item("table", "8-3", "Resumen de experimentos base en validación (media ± desviación estándar cuando procede, n = 3 semillas por experimento)", "87", assets=("reports/tables/summary_results.csv",), references=(pdf_ref, trace_ref)),
                Item("table", "8-4", "Comparativa por modalidad entre SimpleGraspCNN y ResNet18Grasp (mejor época de validación)", "87", assets=("reports/tables/table_ab_comparison_by_modality.csv",), references=(pdf_ref, trace_ref)),
            ),
            artifacts=(
                Item("artifact", "8-A1", "Documento PDF del TFM utilizado como referencia editorial", assets=(pdf_ref,), references=(trace_ref, release_ref)),
            ),
        ),
    )


def main() -> None:
    repo_root = Path(__file__).resolve().parents[2]
    reports_dir = repo_root / "reports"
    chapters_dir = reports_dir / "capitulos"
    pdf_rel = "reports/TFM_Jesus_Lozano_V10.pdf"

    if chapters_dir.exists():
        shutil.rmtree(chapters_dir)
    chapters_dir.mkdir(parents=True, exist_ok=True)

    chapters = build_chapters()

    for chapter in chapters:
        chapter_path = chapters_dir / chapter_dir_name(chapter)
        for folder in (chapter_path, chapter_path / "ilustraciones", chapter_path / "tablas", chapter_path / "artefactos"):
            folder.mkdir(parents=True, exist_ok=True)

        for items in (chapter.illustrations, chapter.tables, chapter.artifacts):
            for item in items:
                folder = item_dir(chapter_path, item.kind)
                copied_assets = copy_assets(repo_root, folder, item)
                if not copied_assets:
                    copied_assets = render_pdf_page(repo_root, folder, item, pdf_rel)
                manifest_path = folder / f"{stem_for(item)}.md"
                write_manifest(manifest_path, chapter, item, copied_assets)

                if chapter.number == "08" and item.number == "8-2":
                    csv_path = folder / f"{stem_for(item)}.csv"
                    build_filtered_exp4_table(repo_root, csv_path)

        (chapter_path / "README.md").write_text(chapter_readme(chapter, chapter_path), encoding="utf-8")

    (reports_dir / "README.md").write_text(reports_readme(chapters), encoding="utf-8")
    (reports_dir / "INVENTARIO_ARTEFACTOS_TFM.md").write_text(inventory_text(chapters), encoding="utf-8")
    write_master_mapping(reports_dir, chapters)


if __name__ == "__main__":
    main()