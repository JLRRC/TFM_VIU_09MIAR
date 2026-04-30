#!/usr/bin/env python3
"""Genera el espacio de trabajo del articulo sin escribir fuera de ARTICULO."""

from __future__ import annotations

import csv
import json
import math
import os
from pathlib import Path
import shutil
import textwrap

import pandas as pd

try:
    import yaml
except Exception:  # pragma: no cover
    yaml = None

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


ROOT = Path(__file__).resolve().parents[2]
ART = Path(__file__).resolve().parent
ANEXOS = ART / "anexos"
RECURSOS = ART / "recursos"
FIG_DIR = RECURSOS / "figuras"
TAB_DIR = RECURSOS / "tablas"
REF_DIR = RECURSOS / "referencias"
DATA_DIR = RECURSOS / "datos_procesados"
DESC_DIR = RECURSOS / "borradores_descartados"

GEN_MARK = "<!-- generado-articulo-tfm -->"


def rel(path: Path) -> str:
    try:
        return str(path.resolve().relative_to(ROOT.resolve()))
    except Exception:
        return str(path)


def ensure_tree() -> None:
    for d in [ANEXOS, FIG_DIR, TAB_DIR, REF_DIR, DATA_DIR, DESC_DIR]:
        d.mkdir(parents=True, exist_ok=True)


def safe_write(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    text = text.rstrip() + "\n"
    if path.exists():
        old = path.read_text(encoding="utf-8", errors="replace")
        if GEN_MARK not in old and old.strip() and old != text:
            backup = path.with_suffix(path.suffix + ".previo")
            if not backup.exists():
                backup.write_text(old, encoding="utf-8")
    path.write_text(f"{GEN_MARK}\n{text}", encoding="utf-8")


def read_yaml(path: Path) -> dict:
    if not path.exists():
        return {}
    if yaml is not None:
        with path.open("r", encoding="utf-8") as fh:
            return yaml.safe_load(fh) or {}
    # Fallback minimo para evitar inventar datos si PyYAML no esta disponible.
    return {"_yaml_no_parseado": True, "_path": rel(path)}


def md_table(rows: list[dict], columns: list[str]) -> str:
    if not rows:
        return "_Sin filas disponibles._"
    header = "| " + " | ".join(columns) + " |"
    sep = "| " + " | ".join(["---"] * len(columns)) + " |"
    body = []
    for row in rows:
        vals = []
        for c in columns:
            val = row.get(c, "")
            vals.append(str(val).replace("\n", "<br>").replace("|", "\\|"))
        body.append("| " + " | ".join(vals) + " |")
    return "\n".join([header, sep, *body])


def fmt(x, pct: bool = False, nd: int = 4) -> str:
    if x is None:
        return "--"
    try:
        f = float(x)
    except Exception:
        return str(x)
    if not math.isfinite(f):
        return "--"
    if pct:
        return f"{f * 100:.1f}%"
    return f"{f:.{nd}f}"


CONFIG_FILES = {
    "EXP1_SIMPLE_RGB": ROOT / "agarre_inteligente/config/exp1_simple_rgb.yaml",
    "EXP2_SIMPLE_RGBD": ROOT / "agarre_inteligente/config/exp2_simple_rgbd.yaml",
    "EXP3_RESNET18_RGB_AUGMENT": ROOT / "agarre_inteligente/config/exp3_resnet18_rgb_augment.yaml",
    "EXP4_RESNET18_RGBD": ROOT / "agarre_inteligente/config/exp4_resnet18_rgbd.yaml",
    "EXP1.1_SIMPLEGRASP_RGB": ROOT / "agarre_inteligente/config/exp1_1_simplegrasp_rgb.yaml",
    "EXP1.2_SIMPLEGRASP_RGBD": ROOT / "agarre_inteligente/config/exp1_2_simplegrasp_rgbd.yaml",
    "EXP_METHOD_V2_RGB": ROOT / "agarre_inteligente/config/exp_methodology_v2.yaml",
    "EXP_TEMPLATE": ROOT / "agarre_inteligente/config/default.yaml",
}

OFFICIAL_EXPS = [
    "EXP1_SIMPLE_RGB",
    "EXP2_SIMPLE_RGBD",
    "EXP3_RESNET18_RGB_AUGMENT",
    "EXP4_RESNET18_RGBD",
]
AUX_EXPS = ["EXP1.1_SIMPLEGRASP_RGB", "EXP1.2_SIMPLEGRASP_RGBD"]


def load_experiments() -> tuple[pd.DataFrame, pd.DataFrame, pd.DataFrame]:
    rows = []
    metrics_frames = []
    best_frames = []
    for exp, cfg_path in CONFIG_FILES.items():
        cfg = read_yaml(cfg_path)
        exp_cfg = cfg.get("experiment", {}) if isinstance(cfg.get("experiment"), dict) else {}
        model_cfg = cfg.get("model", {}) if isinstance(cfg.get("model"), dict) else {}
        data_cfg = cfg.get("data", {}) if isinstance(cfg.get("data"), dict) else {}
        train_cfg = cfg.get("training", {}) if isinstance(cfg.get("training"), dict) else {}
        exp_dir = ROOT / "agarre_inteligente/experiments" / exp
        best_path = exp_dir / "best_epoch_summary.csv"
        metrics_paths = sorted(exp_dir.glob("seed_*/metrics.csv"))
        status = "final" if exp == "EXP3_RESNET18_RGB_AUGMENT" else (
            "oficial" if exp in OFFICIAL_EXPS else (
                "auxiliar" if exp in AUX_EXPS else (
                    "experimental metodologico configurado" if exp == "EXP_METHOD_V2_RGB" else "plantilla"
                )
            )
        )
        row = {
            "experimento": exp,
            "modelo_config": model_cfg.get("name", "--"),
            "clase_real": {
                "SimpleGraspCNN": "SimpleCNN",
                "ResNet18Grasp": "ResNetGrasp",
                "SimpleGrasp": "SimpleGrasp",
            }.get(str(model_cfg.get("name", "")), "--"),
            "modalidad": str(data_cfg.get("modality", "--")).upper().replace("RGBD", "RGB-D"),
            "canales": model_cfg.get("input_channels", "--"),
            "augmentation": data_cfg.get("augmentation", "--"),
            "augmentation_level": data_cfg.get("augmentation_level", "--"),
            "epochs": train_cfg.get("epochs", "--"),
            "criterion": train_cfg.get("criterion", "smooth_l1"),
            "config_file": rel(cfg_path),
            "metrics_csv_por_seed": len(metrics_paths),
            "best_epoch_summary": rel(best_path) if best_path.exists() else "no localizado",
            "aparece_en": "codigo/config/resultados" if best_path.exists() else "codigo/config",
            "estado": status,
        }
        rows.append(row)
        if best_path.exists():
            best = pd.read_csv(best_path)
            best["experiment"] = exp
            best["model"] = row["modelo_config"]
            best["modality"] = row["modalidad"]
            best["status"] = status
            best_frames.append(best)
        for mp in metrics_paths:
            try:
                seed = int(mp.parent.name.split("_")[-1])
            except Exception:
                seed = -1
            m = pd.read_csv(mp)
            m["experiment"] = exp
            m["seed"] = seed
            m["model"] = row["modelo_config"]
            m["modality"] = row["modalidad"]
            m["status"] = status
            metrics_frames.append(m)
    exp_df = pd.DataFrame(rows)
    metrics_df = pd.concat(metrics_frames, ignore_index=True) if metrics_frames else pd.DataFrame()
    best_df = pd.concat(best_frames, ignore_index=True) if best_frames else pd.DataFrame()
    return exp_df, metrics_df, best_df


def summarize(best_df: pd.DataFrame) -> pd.DataFrame:
    if best_df.empty:
        return pd.DataFrame()
    cols = ["val_success", "val_iou", "val_angle_deg", "val_loss", "best_epoch"]
    out = best_df.groupby(["experiment", "model", "modality", "status"], as_index=False).agg(
        n_seeds=("seed", "count"),
        val_success_mean=("val_success", "mean"),
        val_success_std=("val_success", "std"),
        val_iou_mean=("val_iou", "mean"),
        val_iou_std=("val_iou", "std"),
        val_angle_deg_mean=("val_angle_deg", "mean"),
        val_angle_deg_std=("val_angle_deg", "std"),
        val_loss_mean=("val_loss", "mean"),
        val_loss_std=("val_loss", "std"),
        best_epoch_mean=("best_epoch", "mean"),
        best_epoch_std=("best_epoch", "std"),
    )
    return out


MODEL_ROWS = [
    {
        "nombre_codigo": "SimpleGraspCNN",
        "clase_real": "SimpleCNN",
        "archivo_definicion": "agarre_inteligente/src/models/simple_cnn.py",
        "arquitectura": "CNN ligera: tres bloques Conv2d 3x3 + BatchNorm + ReLU + MaxPool, AdaptiveAvgPool2d(1,1), MLP 128-128-5.",
        "entradas": "Tensor imagen 224x224 con 3 canales RGB o 4 canales RGB-D segun input_channels.",
        "salidas": "Vector de 5 parametros: cx, cy, w, h, angle. En dataset se entrenan normalizados; el wrapper ROS decodifica a pixeles/angulo.",
        "uso_real": "Entrenamiento oficial EXP1/EXP2, inferencia CLI generica, cargable por wrapper ROS actual.",
        "estado": "Activo como baseline oficial; el nombre de clase no coincide con el nombre de configuracion.",
        "evidencia": "factory.py construye SimpleCNN si model.name == SimpleGraspCNN; configs EXP1/EXP2; metrics en experiments/EXP1 y EXP2.",
    },
    {
        "nombre_codigo": "ResNet18Grasp",
        "clase_real": "ResNetGrasp",
        "archivo_definicion": "agarre_inteligente/src/models/resnet_variants.py",
        "arquitectura": "torchvision ResNet-18 con fc reemplazada por Dropout + Linear(...,5); conv1 adaptada cuando input_channels != 3.",
        "entradas": "Tensor imagen 224x224 RGB (3 canales) o RGB-D (4 canales).",
        "salidas": "Vector de 5 parametros: cx, cy, w, h, angle.",
        "uso_real": "Entrenamiento oficial EXP3/EXP4, inferencia CLI generica y wrapper ROS. El preset de memoria usa EXP3 seed_0.",
        "estado": "Activo y modelo final documentado para inferencia reproducible.",
        "evidencia": "factory.py construye ResNetGrasp para ResNet18Grasp; panel_tfm_science fija EXP3_RESNET18_RGB_AUGMENT/seed_0 en modo memoria.",
    },
    {
        "nombre_codigo": "SimpleGrasp",
        "clase_real": "SimpleGrasp",
        "archivo_definicion": "agarre_inteligente/src/models/simple_grasp.py",
        "arquitectura": "CNN ligera alineada con diseno teorico: primera Conv2d 7x7 stride 2, bloques 3x3, AdaptiveAvgPool2d(7,7), MLP 128*7*7-256-5.",
        "entradas": "Tensor imagen 224x224 RGB o RGB-D segun input_channels.",
        "salidas": "Vector de 5 parametros normalizados; wrapper ROS aplica clipping para evitar decodificacion divergente.",
        "uso_real": "Experimentos auxiliares EXP1.1/EXP1.2 y pruebas de carga del wrapper ROS.",
        "estado": "Activo como variante auxiliar/posterior; no sustituye resultados oficiales EXP1..EXP4.",
        "evidencia": "configs exp1_1/exp1_2, docs/modelos/simplegrasp.md, test_model_load_exp11.py.",
    },
    {
        "nombre_codigo": "graspnet.models.simple_grasp_cnn.SimpleGraspCNN",
        "clase_real": "No encontrada en el arbol actual",
        "archivo_definicion": "No existe bajo agarre_inteligente/graspnet/models en el workspace actual",
        "arquitectura": "Referencia legacy no materializada como archivo actual.",
        "entradas": "El nodo legacy la invoca como RGB de 3 canales.",
        "salidas": "Vector de 5 parametros segun nodo legacy.",
        "uso_real": "Nodo ROS legacy grasp_inference.py intenta importarla; el wrapper actual usa fallback a src/models.",
        "estado": "Legacy/obsoleto o referencia rota; no debe presentarse como modelo implementado actual.",
        "evidencia": "agarre_ros2_ws/src/tfm_grasping/tfm_grasping/grasp_inference.py importa graspnet.models.simple_grasp_cnn; find no localiza ese modulo.",
    },
]


def save_tables(exp_df: pd.DataFrame, metrics_df: pd.DataFrame, best_df: pd.DataFrame, summary: pd.DataFrame) -> None:
    pd.DataFrame(MODEL_ROWS).to_csv(TAB_DIR / "inventario_modelos.csv", index=False)
    exp_df.to_csv(TAB_DIR / "inventario_experimentos.csv", index=False)
    metrics_df.to_csv(DATA_DIR / "metricas_por_epoca_todos_experimentos.csv", index=False)
    best_df.to_csv(TAB_DIR / "resultados_best_epoch_por_seed.csv", index=False)
    summary.to_csv(TAB_DIR / "resumen_metricas_por_experimento.csv", index=False)
    latency = ROOT / "report/tables/cap5/Tabla_5-3_medicion_de_latencia_de_inferencia_por_experimento_y_dispositivo.csv"
    if latency.exists():
        shutil.copy2(latency, TAB_DIR / "latencia_inferencia_cap5.csv")
    comp = ROOT / "report/tables/cap5/Tabla_5-4_comparativa_por_modalidad_entre_simplegraspcnn_y_resnet18grasp.csv"
    if comp.exists():
        shutil.copy2(comp, TAB_DIR / "comparativa_modalidad_simplecnn_resnet18.csv")
    validated = ROOT / "report/metrics/validated/chapter5_experiment_summary_validated.csv"
    if validated.exists():
        shutil.copy2(validated, TAB_DIR / "chapter5_experiment_summary_validated.csv")
    prepost = ROOT / "report/metrics/aggregated/chapter5_pre_vs_post_retrain_comparison.csv"
    if prepost.exists():
        shutil.copy2(prepost, DATA_DIR / "chapter5_pre_vs_post_retrain_comparison.csv")


def style_ax(ax):
    ax.grid(True, alpha=0.25)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)


def plot_all(metrics_df: pd.DataFrame, best_df: pd.DataFrame, summary: pd.DataFrame) -> list[dict]:
    figs: list[dict] = []
    if metrics_df.empty:
        return figs

    colors = {
        "EXP1_SIMPLE_RGB": "#4477AA",
        "EXP2_SIMPLE_RGBD": "#66CCEE",
        "EXP3_RESNET18_RGB_AUGMENT": "#228833",
        "EXP4_RESNET18_RGBD": "#CCBB44",
        "EXP1.1_SIMPLEGRASP_RGB": "#EE6677",
        "EXP1.2_SIMPLEGRASP_RGBD": "#AA3377",
    }

    for exp in sorted(metrics_df["experiment"].unique()):
        df = metrics_df[metrics_df["experiment"] == exp]
        fig, ax = plt.subplots(figsize=(7.2, 4.2), dpi=160)
        for seed, sdf in df.groupby("seed"):
            ax.plot(sdf["epoch"], sdf["train_loss"], lw=1.2, alpha=0.55, color="#888888")
            ax.plot(sdf["epoch"], sdf["val_loss"], lw=1.8, label=f"seed {seed} val", color=colors.get(exp, "#333333"), alpha=0.75)
        ax.set_title(f"Curvas de perdida - {exp}")
        ax.set_xlabel("Epoca")
        ax.set_ylabel("Loss")
        style_ax(ax)
        ax.legend(fontsize=7, ncol=2)
        fn = f"curvas_loss_{exp}.png"
        fig.tight_layout()
        fig.savefig(FIG_DIR / fn)
        plt.close(fig)
        figs.append({
            "archivo": fn,
            "representa": "Curvas train_loss y val_loss por seed.",
            "fuente": "agarre_inteligente/experiments/*/seed_*/metrics.csv",
            "experimento_modelo": f"{exp}",
            "apta_articulo": "si, si se simplifica visualmente segun limite de paginas",
            "retoque": "posible reduccion a paneles agregados",
        })

    for metric, ylabel, title, fn in [
        ("val_success", "Grasp success", "Evolucion de success en validacion", "evolucion_val_success_todos.png"),
        ("val_iou", "IoU medio", "Evolucion de IoU en validacion", "evolucion_val_iou_todos.png"),
        ("val_angle_deg", "Error angular medio (deg)", "Evolucion del error angular", "evolucion_val_angle_todos.png"),
    ]:
        fig, ax = plt.subplots(figsize=(7.6, 4.4), dpi=160)
        mean_df = metrics_df.groupby(["experiment", "epoch"], as_index=False)[metric].mean()
        for exp, sdf in mean_df.groupby("experiment"):
            ax.plot(sdf["epoch"], sdf[metric], lw=2, label=exp, color=colors.get(exp))
        ax.set_title(title)
        ax.set_xlabel("Epoca")
        ax.set_ylabel(ylabel)
        style_ax(ax)
        ax.legend(fontsize=7)
        fig.tight_layout()
        fig.savefig(FIG_DIR / fn)
        plt.close(fig)
        figs.append({
            "archivo": fn,
            "representa": f"Media por epoca de {metric} para experimentos con metricas disponibles.",
            "fuente": "agarre_inteligente/experiments/*/seed_*/metrics.csv",
            "experimento_modelo": "EXP1..EXP4 + EXP1.1/EXP1.2 si hay metricas",
            "apta_articulo": "si",
            "retoque": "revisar densidad de leyenda",
        })

    for metric, ylabel, title, fn in [
        ("val_success_mean", "Grasp success medio", "Resultados finales: success", "best_epoch_success_barras.png"),
        ("val_iou_mean", "IoU medio", "Resultados finales: IoU", "best_epoch_iou_barras.png"),
        ("val_angle_deg_mean", "Error angular medio (deg)", "Resultados finales: error angular", "best_epoch_angle_barras.png"),
        ("val_loss_mean", "Val loss media", "Resultados finales: val_loss", "best_epoch_loss_barras.png"),
    ]:
        sdf = summary.sort_values(metric, ascending=(metric != "val_angle_deg_mean"))
        fig, ax = plt.subplots(figsize=(7.6, 4.2), dpi=160)
        ax.bar(sdf["experiment"], sdf[metric], yerr=sdf[metric.replace("_mean", "_std")] if metric.replace("_mean", "_std") in sdf else None, color=[colors.get(e, "#777777") for e in sdf["experiment"]])
        ax.set_title(title)
        ax.set_ylabel(ylabel)
        ax.tick_params(axis="x", rotation=35, labelsize=7)
        style_ax(ax)
        fig.tight_layout()
        fig.savefig(FIG_DIR / fn)
        plt.close(fig)
        figs.append({
            "archivo": fn,
            "representa": f"Media y desviacion por seed de {metric}.",
            "fuente": "best_epoch_summary.csv por experimento",
            "experimento_modelo": "todos los experimentos ejecutados",
            "apta_articulo": "si",
            "retoque": "posible acortar nombres de experimentos",
        })

    official = summary[summary["experiment"].isin(OFFICIAL_EXPS)].copy()
    if not official.empty:
        pivot = official.pivot_table(index="modality", columns="model", values="val_success_mean")
        fig, ax = plt.subplots(figsize=(6.6, 4.0), dpi=160)
        pivot.plot(kind="bar", ax=ax, color=["#4477AA", "#228833"])
        ax.set_title("Comparacion por modalidad: SimpleGraspCNN vs ResNet18Grasp")
        ax.set_ylabel("Grasp success medio")
        ax.tick_params(axis="x", rotation=0)
        style_ax(ax)
        fig.tight_layout()
        fn = "comparacion_modalidad_success.png"
        fig.savefig(FIG_DIR / fn)
        plt.close(fig)
        figs.append({
            "archivo": fn,
            "representa": "Comparacion de success final entre modelo ligero y ResNet18 por modalidad.",
            "fuente": "report/tables/cap5/Tabla_5-4 y best_epoch_summary oficiales",
            "experimento_modelo": "EXP1..EXP4",
            "apta_articulo": "si",
            "retoque": "no imprescindible",
        })

    latency_path = TAB_DIR / "latencia_inferencia_cap5.csv"
    if latency_path.exists():
        lat = pd.read_csv(latency_path)
        fig, ax = plt.subplots(figsize=(7.4, 4.2), dpi=160)
        for device, sdf in lat.groupby("device"):
            ax.bar([f"{e}\n{device}" for e in sdf["experiment"]], sdf["latency_ms_mean"], label=device)
        ax.set_yscale("log")
        ax.set_title("Latencia media de inferencia")
        ax.set_ylabel("ms (escala log)")
        ax.tick_params(axis="x", rotation=35, labelsize=7)
        style_ax(ax)
        fig.tight_layout()
        fn = "latencia_inferencia_log.png"
        fig.savefig(FIG_DIR / fn)
        plt.close(fig)
        figs.append({
            "archivo": fn,
            "representa": "Latencia media CPU/CUDA para batch 1.",
            "fuente": "report/tables/cap5/Tabla_5-3_medicion_de_latencia_de_inferencia_por_experimento_y_dispositivo.csv",
            "experimento_modelo": "EXP1..EXP4",
            "apta_articulo": "si",
            "retoque": "quizas separar CPU y CUDA si el venue exige legibilidad en B/N",
        })

        fig, ax1 = plt.subplots(figsize=(7.4, 4.2), dpi=160)
        cpu = lat[lat["device"] == "cpu"].drop_duplicates("experiment")
        ax1.bar(cpu["experiment"], cpu["n_params"] / 1e6, color="#999933")
        ax1.set_ylabel("Parametros (millones)")
        ax1.set_title("Tamano de modelo por experimento")
        ax1.tick_params(axis="x", rotation=35, labelsize=7)
        style_ax(ax1)
        fig.tight_layout()
        fn = "tamano_modelo_parametros.png"
        fig.savefig(FIG_DIR / fn)
        plt.close(fig)
        figs.append({
            "archivo": fn,
            "representa": "Numero de parametros por checkpoint medido en benchmark.",
            "fuente": "Tabla de latencia capitulo 5",
            "experimento_modelo": "EXP1..EXP4",
            "apta_articulo": "si",
            "retoque": "no imprescindible",
        })

    prepost_path = DATA_DIR / "chapter5_pre_vs_post_retrain_comparison.csv"
    if prepost_path.exists():
        pp = pd.read_csv(prepost_path)
        delta_col = "delta_val_success"
        if delta_col not in pp.columns:
            delta_col = "delta_val_success_mean"
        if delta_col in pp.columns:
            fig, ax = plt.subplots(figsize=(7.2, 4.0), dpi=160)
            ax.bar(pp["experiment"], pp[delta_col], color="#AA4499")
            ax.axhline(0, color="black", lw=0.8)
            ax.set_title("Cambio de success tras retrain documentado")
            ax.set_ylabel("Delta val_success")
            ax.tick_params(axis="x", rotation=35, labelsize=7)
            style_ax(ax)
            fig.tight_layout()
            fn = "delta_pre_post_retrain_success.png"
            fig.savefig(FIG_DIR / fn)
            plt.close(fig)
            figs.append({
                "archivo": fn,
                "representa": "Diferencia de val_success entre snapshot previo y resultados validados.",
                "fuente": "report/metrics/aggregated/chapter5_pre_vs_post_retrain_comparison.csv",
                "experimento_modelo": "EXP1..EXP4",
                "apta_articulo": "solo si se explica como control de reproducibilidad, no como resultado principal",
                "retoque": "decidir si incluir o dejar en material suplementario",
            })

    pd.DataFrame(figs).to_csv(DATA_DIR / "inventario_figuras_generadas.csv", index=False)
    return figs


def rows_for_md(summary: pd.DataFrame) -> list[dict]:
    rows = []
    for _, r in summary.sort_values("experiment").iterrows():
        rows.append({
            "experimento": r["experiment"],
            "modelo": r["model"],
            "modalidad": r["modality"],
            "estado": r["status"],
            "success": f"{fmt(r['val_success_mean'])} +/- {fmt(r['val_success_std'])}",
            "IoU": f"{fmt(r['val_iou_mean'])} +/- {fmt(r['val_iou_std'])}",
            "error angular": f"{fmt(r['val_angle_deg_mean'])} +/- {fmt(r['val_angle_deg_std'])}",
            "val_loss": f"{fmt(r['val_loss_mean'])} +/- {fmt(r['val_loss_std'])}",
        })
    return rows


def build_markdown(exp_df: pd.DataFrame, best_df: pd.DataFrame, summary: pd.DataFrame, figs: list[dict]) -> None:
    top = summary.sort_values("val_success_mean", ascending=False).iloc[0].to_dict() if not summary.empty else {}
    official = summary[summary["experiment"].isin(OFFICIAL_EXPS)]
    final_exp = "EXP3_RESNET18_RGB_AUGMENT"
    final_row = summary[summary["experiment"] == final_exp].iloc[0].to_dict() if final_exp in set(summary["experiment"]) else {}

    result_table = md_table(rows_for_md(summary), ["experimento", "modelo", "modalidad", "estado", "success", "IoU", "error angular", "val_loss"])
    model_table = md_table(MODEL_ROWS, ["nombre_codigo", "clase_real", "archivo_definicion", "arquitectura", "entradas", "salidas", "uso_real", "estado", "evidencia"])
    exp_table = md_table(exp_df.to_dict("records"), ["experimento", "modelo_config", "clase_real", "modalidad", "canales", "augmentation", "augmentation_level", "epochs", "criterion", "config_file", "metrics_csv_por_seed", "best_epoch_summary", "estado"])
    figs_table = md_table(figs, ["archivo", "representa", "fuente", "experimento_modelo", "apta_articulo", "retoque"])

    safe_write(ART / "README.md", f"""
# Articulo cientifico derivado del TFM

Este directorio es el espacio de trabajo para convertir el proyecto de master en un articulo cientifico. Todo el contenido generado aqui procede de fuentes internas del workspace: codigo, configuraciones, metricas, tablas, figuras y evidencias ya existentes.

## Estado actual

- Modelos inventariados: `SimpleGraspCNN`/`SimpleCNN`, `ResNet18Grasp`/`ResNetGrasp`, `SimpleGrasp` y referencias legacy no materializadas.
- Experimentos ejecutados con metricas: `EXP1_SIMPLE_RGB`, `EXP2_SIMPLE_RGBD`, `EXP3_RESNET18_RGB_AUGMENT`, `EXP4_RESNET18_RGBD`, `EXP1.1_SIMPLEGRASP_RGB`, `EXP1.2_SIMPLEGRASP_RGBD`.
- Configuraciones adicionales localizadas: `EXP_METHOD_V2_RGB` y `EXP_TEMPLATE`.
- Modelo final documentado para inferencia reproducible: `{final_exp}` con `ResNet18Grasp`, seed 0 en el preset de memoria.
- Figuras generadas para el articulo: {len(figs)} PNG en `recursos/figuras/`.

## Proximos pasos

1. Elegir venue y plantilla para ajustar extension, estructura y estilo bibliografico.
2. Decidir si el articulo se centra solo en `EXP1..EXP4` o si incorpora `EXP1.1/EXP1.2` como material suplementario.
3. Revisar con el tutor las discrepancias documentadas en `06_preguntas_abiertas.md`.
4. Seleccionar 4-6 figuras finales desde `05_figuras_y_tablas.md`.
5. Convertir `02_articulo_borrador.md` a la plantilla final.
""")

    safe_write(ART / "00_plan_trabajo.md", """
# Plan de trabajo para convertir el TFM en articulo

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
""")

    safe_write(ART / "01_resumen_congreso.md", f"""
# Resumen de congreso - borrador inicial

Se presenta una comparacion reproducible de modelos convolucionales para prediccion de rectangulos de agarre 2D/2.5D a partir de imagenes RGB y RGB-D, integrada en un flujo funcional de percepcion y ejecucion robotica con ROS 2, Gazebo, MoveIt 2 y un manipulador UR5 con pinza RG2. El estudio parte de un pipeline experimental sobre el dataset Cornell, con particion object-wise, tres semillas por experimento y metricas de validacion basadas en `grasp success`, IoU y error angular.

Se evaluaron dos familias oficiales: una CNN ligera configurada como `SimpleGraspCNN` e implementada como `SimpleCNN`, y una variante profunda `ResNet18Grasp` implementada sobre ResNet-18. Los experimentos oficiales comparan RGB frente a RGB-D y configuraciones con y sin augmentation. Los resultados validados muestran que `{final_exp}` alcanza un `val_success` medio de {fmt(final_row.get('val_success_mean'))} +/- {fmt(final_row.get('val_success_std'))}, con IoU medio de {fmt(final_row.get('val_iou_mean'))} +/- {fmt(final_row.get('val_iou_std'))} y error angular medio de {fmt(final_row.get('val_angle_deg_mean'))} +/- {fmt(final_row.get('val_angle_deg_std'))} grados. La comparacion por modalidad indica una ventaja clara de `ResNet18Grasp` frente a la CNN ligera tanto en RGB como en RGB-D, mientras que la inclusion de profundidad no mejora de forma uniforme el rendimiento frente a la variante RGB con augmentation.

Como contribucion adicional, el proyecto conserva una implementacion auxiliar `SimpleGrasp` alineada con la arquitectura teorica descrita en la memoria, junto con una variante metodologica posterior que introduce perdida angular periodica e IoU orientada. Estas extensiones se documentan separadamente para no mezclar resultados oficiales y resultados auxiliares. El trabajo aporta asi una base experimental trazable, codigo de entrenamiento e inferencia verificable, y una integracion funcional del modelo seleccionado en un escenario robotico simulado.
""")

    safe_write(ART / "02_articulo_borrador.md", f"""
# Titulo

Comparacion reproducible de CNN ligeras y ResNet-18 para prediccion de agarres 2D/2.5D con integracion ROS 2

## Resumen

Este articulo deriva de un trabajo de fin de master centrado en la deteccion de poses de agarre mediante vision y aprendizaje profundo. Se comparan modelos convolucionales para regresion de rectangulos de agarre en formato Cornell, usando imagenes RGB y RGB-D, particion object-wise y tres semillas por configuracion. El pipeline incluye entrenamiento, evaluacion, benchmark de latencia e integracion funcional en ROS 2/Gazebo/MoveIt 2 para un UR5 con pinza RG2. Los resultados oficiales muestran que `ResNet18Grasp` con RGB y augmentation (`EXP3_RESNET18_RGB_AUGMENT`) alcanza el mejor rendimiento medio entre los experimentos consolidados: `val_success` {fmt(final_row.get('val_success_mean'))} +/- {fmt(final_row.get('val_success_std'))}, IoU {fmt(final_row.get('val_iou_mean'))} +/- {fmt(final_row.get('val_iou_std'))} y error angular {fmt(final_row.get('val_angle_deg_mean'))} +/- {fmt(final_row.get('val_angle_deg_std'))} grados. El trabajo tambien documenta discrepancias entre la arquitectura ligera teorica y la baseline historica usada en resultados, preservando la trazabilidad entre codigo, configuraciones y artefactos experimentales.

## Palabras clave

agarre robotico; CNN; ResNet-18; RGB-D; Cornell grasping dataset; ROS 2; reproducibilidad

## Introduccion

La prediccion de agarres a partir de imagenes es una tarea central en manipulacion robotica. En escenarios no estructurados, una formulacion frecuente consiste en estimar un rectangulo orientado de agarre definido por centro, dimensiones y angulo. Este enfoque reduce la complejidad frente a formulaciones 6-DoF completas y permite evaluar modelos mediante protocolos derivados del Cornell Grasping Dataset.

El proyecto del que deriva este articulo implementa un pipeline completo: preparacion de datos, definicion de modelos, entrenamiento multisemilla, evaluacion, analisis de latencia e integracion de inferencia en un entorno ROS 2/Gazebo/MoveIt 2. La pregunta principal no es proponer una arquitectura nueva, sino medir de forma trazable que configuracion resulta mas adecuada dentro de un sistema robotico reproducible.

## Estado del arte

Pendiente de cerrar con referencias formales del venue. La redaccion deberia cubrir, como minimo:

- formulacion Cornell de rectangulos de agarre;
- redes densas tipo GGCNN y variantes RGB-D;
- enfoques basados en CNN profundas y transferencia;
- datasets Cornell, Jacquard y GraspNet/Acronym como contexto;
- integracion de percepcion con ROS/MoveIt para manipulacion.

## Metodologia

El pipeline experimental usa configuraciones YAML versionadas en `agarre_inteligente/config/`. Cada experimento define modelo, modalidad de entrada, augmentation, optimizador, epocas, batch size, funcion de perdida y rutas de datos. El entrenamiento se ejecuta mediante `scripts/train.py`, que construye el modelo desde `src/models/factory.py`, carga el dataset Cornell procesado, aplica transformaciones de entrenamiento/validacion y guarda metricas por epoca, checkpoints y `config_snapshot.yaml`.

La evaluacion oficial usa `SmoothL1Loss` para entrenamiento y metricas Cornell implementadas en el codigo historico. Existe una variante metodologica posterior con `GraspLoss` e IoU orientada, pero no sustituye los resultados oficiales.

## Modelos

{model_table}

## Protocolo experimental

Los experimentos oficiales son `EXP1..EXP4`. Cada uno se ejecuta con tres seeds. Las metricas principales son `val_success`, `val_iou`, `val_angle_deg` y `val_loss` en la mejor epoca de validacion. Tambien hay mediciones de latencia CPU/CUDA para batch 1.

{exp_table}

## Resultados

{result_table}

El mejor resultado medio entre los experimentos ejecutados corresponde a `{top.get('experiment', '--')}` con `val_success` {fmt(top.get('val_success_mean'))}. En el bloque oficial del TFM, el caso final documentado para inferencia es `{final_exp}`.

## Discusion

Los resultados indican que la familia ResNet-18 proporciona una mejora clara frente a la CNN ligera oficial. La variante RGB con augmentation (`EXP3`) supera a la configuracion RGB-D de ResNet (`EXP4`) en success medio, aunque `EXP4` conserva un `val_loss` competitivo. En la CNN ligera, RGB-D con augmentation (`EXP2`) mejora a RGB sin augmentation (`EXP1`), lo que sugiere que la utilidad de profundidad y augmentation depende de arquitectura y protocolo.

El articulo debe explicar que `SimpleGraspCNN` es un nombre de configuracion: la clase real en el codigo actual es `SimpleCNN`. Tambien debe separar los resultados oficiales de `EXP1..EXP4` de `EXP1.1/EXP1.2`, que implementan la arquitectura ligera teorica `SimpleGrasp`.

## Limitaciones

- Dataset Cornell y validacion offline; no demuestra generalizacion universal.
- Tres semillas por experimento; analisis estadistico limitado.
- Metricas oficiales historicas no usan la variante orientada posterior.
- Integracion robotica funcional en simulacion; no se documenta validacion fisica en robot real.
- La profundidad no produce una mejora uniforme en todos los modelos.

## Conclusiones

El proyecto ofrece una comparacion reproducible y trazable de modelos CNN para prediccion de agarres 2D/2.5D. La mejor configuracion oficial es `ResNet18Grasp` RGB con augmentation. La infraestructura conserva codigo, configuraciones, metricas, checkpoints e integracion ROS 2 suficientes para construir un articulo centrado en reproducibilidad experimental e integracion aplicada.

## Trabajo futuro

- Evaluar con IoU orientada y perdida angular periodica de forma oficial.
- Ampliar datasets y protocolos de validacion.
- Validar en robot fisico o con mayor diversidad de escenas simuladas.
- Comparar con modelos densos de mapas de calidad/angulo/apertura.
- Reducir latencia y tamano de modelos para ejecucion embarcada.

## Referencias preliminares

Pendiente de completar con bibliografia formal. Candidatas: Cornell Grasping Dataset; GGCNN; Dex-Net; Jacquard; GraspNet; MoveIt 2; ROS 2; ResNet.
""")

    titles = [
        "Comparacion reproducible de CNN para deteccion de agarres 2D/2.5D en un pipeline ROS 2",
        "Evaluacion trazable de Simple CNN y ResNet-18 para prediccion de rectangulos de agarre",
        "De Cornell a ROS 2: comparacion experimental de modelos CNN para agarre robotico",
        "Analisis reproducible de arquitecturas RGB y RGB-D para grasping basado en rectangulos",
        "ResNet-18 frente a CNN ligeras para prediccion de agarres: resultados y despliegue en simulacion",
        "Benchmark reproducible de modelos de grasping 2D con integracion Gazebo/MoveIt 2",
        "Pipeline verificable para entrenamiento e inferencia de agarres roboticos con CNN",
        "Comparacion de modalidades RGB y RGB-D en regresion de rectangulos de agarre",
        "Lecciones de reproducibilidad en grasping visual: modelos, metricas e integracion robotica",
        "Evaluacion multisemilla de CNN para grasping 2D/2.5D sobre Cornell",
        "Integracion funcional de ResNet-18 para inferencia de agarres en UR5 simulado",
        "Arquitecturas convolucionales para agarre robotico: comparacion, latencia y trazabilidad",
        "Un estudio reproducible de SimpleGraspCNN y ResNet18Grasp para manipulacion robotica",
        "Prediccion de agarres con aprendizaje profundo: comparacion experimental e integracion ROS 2",
        "Del prototipo de TFM al articulo: evidencia reproducible en grasping visual con CNN",
        "Evaluacion de augmentation y profundidad para modelos de grasping 2D",
        "Grasping visual con UR5 simulado: comparacion de modelos y seleccion de checkpoint final",
    ]
    safe_write(ART / "03_titulos_posibles.md", "# Titulos posibles\n\n" + "\n".join(f"{i+1}. {t}" for i, t in enumerate(titles)))

    safe_write(ART / "04_aportaciones_clave.md", """
# Aportaciones clave

## Aportacion real

- Comparacion reproducible de dos familias oficiales de modelos (`SimpleGraspCNN`/`SimpleCNN` y `ResNet18Grasp`/`ResNetGrasp`) en RGB y RGB-D.
- Evidencia multisemilla con metricas por epoca, mejor epoca, latencia y resumen agregado.
- Integracion funcional de inferencia en ROS 2/Gazebo/MoveIt 2 con seleccion de checkpoint final.
- Trazabilidad explicita entre memoria, codigo, configuraciones y resultados.

## Novedad defendible

La novedad defendible no es una arquitectura nueva, sino la combinacion de comparacion reproducible, curacion de artefactos, analisis de trade-offs rendimiento/latencia y despliegue funcional en un pipeline robotico simulado.

## Que no debe prometer el articulo

- No prometer rendimiento superior al estado del arte.
- No afirmar validacion en robot fisico si no se aporta evidencia directa.
- No mezclar metricas oficiales historicas con la variante metodologica posterior.
- No presentar `SimpleGraspCNN` como clase Python real; es nombre de configuracion que instancia `SimpleCNN`.

## Mensaje principal recomendado

`ResNet18Grasp` RGB con augmentation ofrece el mejor equilibrio de rendimiento dentro del bloque oficial, mientras que el workspace aporta trazabilidad suficiente para reproducir y auditar la comparacion completa.
""")

    safe_write(ART / "05_figuras_y_tablas.md", f"""
# Figuras y tablas

## Figuras generadas para el articulo

{figs_table}

## Tablas nuevas generadas

- `recursos/tablas/inventario_modelos.csv`: modelos implementados y referencias legacy.
- `recursos/tablas/inventario_experimentos.csv`: configs y experimentos ejecutados/localizados.
- `recursos/tablas/resultados_best_epoch_por_seed.csv`: resultados por seed en best epoch.
- `recursos/tablas/resumen_metricas_por_experimento.csv`: medias y desviaciones por experimento.
- `recursos/tablas/latencia_inferencia_cap5.csv`: copia curada de la tabla de latencia del capitulo 5.
- `recursos/tablas/comparativa_modalidad_simplecnn_resnet18.csv`: comparativa oficial por modalidad.

## Figuras existentes reutilizables

- `report/figures/cap5/Ilustracion_5-10_exito_final_de_agarre_en_validacion_agregado_por_experimento.png`
- `report/figures/cap5/Ilustracion_5-11_iou_medio_final_en_validacion_agregado_por_experimento.png`
- `report/figures/cap5/Ilustracion_5-12_error_angular_medio_final_en_validacion_agregado_por_experimento.png`
- `report/figures/cap5/Ilustracion_5-17_resultado_de_inferencia_del_modelo_exp3_resnet18_rgb_augment_sobre_la_imagen_sim.png`
- `report/figures/cap5/Ilustracion_5-18_evidencia_funcional_adicional_del_pipeline_percepcion_publicacion_consumo_en_ros.png`

No se han copiado esas figuras para evitar duplicar artefactos pesados; estan inventariadas como fuentes reutilizables.
""")

    safe_write(ART / "06_preguntas_abiertas.md", """
# Preguntas abiertas

- Venue y formato: conferencia, revista, resumen extendido o workshop.
- Autores, afiliaciones, orden de autores y posible anonimato.
- Confirmar si el articulo debe centrarse exclusivamente en `EXP1..EXP4` o mencionar `EXP1.1/EXP1.2` como extension auxiliar.
- Decidir como explicar la discrepancia entre metrica oficial axis-aligned y formulacion orientada posterior.
- Revisar si hay presentacion final en el workspace no localizada explicitamente con extension conocida.
- Confirmar si las figuras generadas deben pasar a estilo del venue: B/N, vectorial, dos columnas, tamanos de fuente.
- Completar referencias bibliograficas formales.
- Decidir si incluir latencia CUDA/CPU en texto principal o suplemento.
- Verificar si hay evidencia suficiente para describir el pipeline robotico como "validacion funcional" sin sobredimensionarlo.
""")

    safe_write(ART / "07_revision_tutor.md", """
# Revision tutor

## Comentarios pendientes

| Fecha | Seccion | Comentario | Accion | Estado |
|---|---|---|---|---|
| pendiente | pendiente | pendiente | pendiente | abierto |

## Puntos que conviene validar con el tutor

- Mensaje central del articulo.
- Inclusión o exclusion de `EXP1.1/EXP1.2`.
- Nivel de detalle sobre ROS 2/Gazebo/MoveIt 2.
- Venue objetivo y extension.
- Tratamiento de discrepancias entre memoria y codigo.
""")

    safe_write(ART / "08_version_final_envio.md", """
# Checklist final de envio

- [ ] Venue seleccionado y plantilla aplicada.
- [ ] Autores y afiliaciones confirmados.
- [ ] Anonimato revisado si aplica.
- [ ] Todas las cifras del texto verificadas contra CSV interno.
- [ ] Figuras finales exportadas en formato requerido.
- [ ] Tablas reducidas al limite de paginas.
- [ ] Referencias en formato del venue.
- [ ] Limitaciones redactadas sin promesas excesivas.
- [ ] Repositorio/codigo citado segun politica del venue.
- [ ] Revision final del tutor incorporada.
- [ ] PDF compilado sin errores.
- [ ] Material suplementario preparado si se usa.
""")

    sources = [
        "agarre_inteligente/config/*.yaml",
        "agarre_inteligente/src/models/*.py",
        "agarre_inteligente/scripts/train.py",
        "agarre_inteligente/scripts/predict.py",
        "agarre_inteligente/src/data/grasp_dataset.py",
        "agarre_inteligente/src/data/transforms.py",
        "agarre_inteligente/experiments/*/seed_*/metrics.csv",
        "agarre_inteligente/experiments/*/best_epoch_summary.csv",
        "report/metrics/validated/chapter5_experiment_summary_validated.csv",
        "report/tables/cap5/*.csv",
        "agarre_ros2_ws/src/tfm_grasping/tfm_grasping/model.py",
        "agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_tfm_science.py",
        "agarre_inteligente/docs/modelos/*.md",
        "report/evidence/exp1_1_exp1_2_theoretical_implementation_trace_20260415.md",
    ]
    safe_write(ANEXOS / "notas_fuentes.md", "# Notas de fuentes internas\n\n" + "\n".join(f"- `{s}`" for s in sources) + "\n\nPrioridad aplicada: resultados y artefactos experimentales; codigo/configs; presentacion si aparece; memoria/documentacion.")

    safe_write(ANEXOS / "resultados_clave.md", f"""
# Resultados clave

{result_table}

## Lectura principal

- Mejor resultado medio ejecutado: `{top.get('experiment', '--')}` con `val_success` {fmt(top.get('val_success_mean'))}.
- Caso final documentado para inferencia reproducible: `{final_exp}`.
- `ResNet18Grasp` supera a `SimpleGraspCNN` en las dos modalidades oficiales.
- En ResNet, RGB con augmentation (`EXP3`) supera a RGB-D sin augmentation (`EXP4`) en `val_success` medio.
- En la CNN ligera oficial, RGB-D con augmentation (`EXP2`) supera a RGB sin augmentation (`EXP1`).

## Fuentes

- `agarre_inteligente/experiments/*/best_epoch_summary.csv`
- `report/metrics/validated/chapter5_experiment_summary_validated.csv`
- `report/tables/cap5/Tabla_5-2_resumen_de_metricas_finales_por_experimento_en_validacion.csv`
""")

    safe_write(ANEXOS / "metodologia_resumida.md", """
# Metodologia resumida

1. Preparacion del dataset Cornell procesado con particion object-wise.
2. Definicion de experimentos mediante YAML.
3. Construccion del modelo con `src/models/factory.py`.
4. Carga de `GraspDataset` en modalidad RGB o RGB-D.
5. Transformaciones: resize a 224x224; augmentation opcional con flip horizontal, rotacion y color jitter.
6. Entrenamiento con Adam y `SmoothL1Loss` en los experimentos oficiales.
7. Registro de metricas por epoca y seleccion de mejor epoca.
8. Evaluacion por `val_success`, `val_iou`, `val_angle_deg` y `val_loss`.
9. Benchmark de latencia en CPU/CUDA para batch 1.
10. Integracion de checkpoint seleccionado en wrapper ROS 2 para inferencia funcional.
""")

    safe_write(ANEXOS / "limitaciones_y_trabajo_futuro.md", """
# Limitaciones y trabajo futuro

## Limitaciones

- Evaluacion principal sobre Cornell; no cubre diversidad amplia de datasets.
- Tres semillas por experimento, suficientes para trazabilidad pero limitadas para inferencia estadistica fuerte.
- Resultados oficiales calculados con evaluador historico; la variante orientada posterior no reescribe las tablas oficiales.
- Integracion demostrada en simulacion, no como ensayo fisico sistematico.
- RGB-D no mejora de forma uniforme; requiere analisis adicional.

## Trabajo futuro

- Reentrenar bloque oficial con `GraspLoss` e IoU orientada.
- Comparar con arquitecturas densas modernas.
- Evaluar transferencia a Jacquard/GraspNet u otros datasets.
- Validar en robot fisico y medir tasa de agarre real.
- Añadir analisis estadistico formal y ablation de augmentation/profundidad.
""")

    safe_write(ANEXOS / "inventario_modelos.md", f"""
# Inventario de modelos implementados

{model_table}

## Observaciones

- `SimpleGraspCNN` es nombre de configuracion; la clase real actual es `SimpleCNN`.
- `ResNet18Grasp` es nombre de configuracion; la clase real actual es `ResNetGrasp`.
- `SimpleGrasp` esta implementado y entrenado en experimentos auxiliares.
- Las referencias `graspnet.models.*` en el wrapper ROS son compatibilidad/legacy; no existe un paquete `graspnet/models` actual en el arbol inspeccionado.
""")

    safe_write(ANEXOS / "inventario_experimentos.md", f"""
# Inventario de experimentos

{exp_table}

## Estados usados

- `final`: caso documentado para inferencia reproducible del articulo/TFM.
- `oficial`: experimento del bloque consolidado `EXP1..EXP4`.
- `auxiliar`: experimento posterior que implementa la arquitectura teorica `SimpleGrasp`.
- `experimental metodologico configurado`: YAML localizado sin resultados ejecutados en `agarre_inteligente/experiments`.
- `plantilla`: configuracion base no interpretable como resultado experimental.
""")

    safe_write(ANEXOS / "trazabilidad_articulo.md", f"""
# Trazabilidad de afirmaciones del articulo

| Afirmacion | Fuente interna | Estado |
|---|---|---|
| Existen tres familias implementadas actuales: SimpleCNN, ResNetGrasp y SimpleGrasp. | `agarre_inteligente/src/models/*.py`, `src/models/factory.py` | verificado |
| `SimpleGraspCNN` instancia `SimpleCNN`. | `agarre_inteligente/src/models/factory.py` | verificado |
| `ResNet18Grasp` instancia `ResNetGrasp`. | `agarre_inteligente/src/models/factory.py` | verificado |
| Los experimentos oficiales son EXP1..EXP4. | `agarre_inteligente/README.md`, `report/metrics/validated/chapter5_experiment_summary_validated.csv` | verificado |
| EXP1.1/EXP1.2 son auxiliares y no sustituyen resultados oficiales. | `report/evidence/exp1_1_exp1_2_theoretical_implementation_trace_20260415.md`, `agarre_inteligente/README.md` | verificado |
| El preset de memoria fija EXP3 seed_0. | `agarre_ros2_ws/src/ur5_qt_panel/ur5_qt_panel/panel_tfm_science.py` | verificado |
| Mejor modelo oficial por success medio: `{final_exp}`. | `agarre_inteligente/experiments/EXP3_RESNET18_RGB_AUGMENT/best_epoch_summary.csv`, `report/metrics/validated/chapter5_experiment_summary_validated.csv` | verificado |
| Las metricas oficiales usan SmoothL1Loss y evaluador historico. | `agarre_inteligente/scripts/train.py`, `agarre_inteligente/src/evaluation/evaluator.py` | verificado |
| Existe variante metodologica posterior con GraspLoss/EvaluatorOriented. | `agarre_inteligente/src/training/losses.py`, `agarre_inteligente/src/evaluation/evaluator.py`, `config/exp_methodology_v2.yaml` | verificado como configuracion, no resultado oficial |
""")


def main() -> None:
    ensure_tree()
    exp_df, metrics_df, best_df = load_experiments()
    summary = summarize(best_df)
    save_tables(exp_df, metrics_df, best_df, summary)
    figs = plot_all(metrics_df, best_df, summary)
    build_markdown(exp_df, best_df, summary, figs)
    manifest = {
        "root": rel(ROOT),
        "articulo_dir": rel(ART),
        "n_experimentos_configurados": int(len(exp_df)),
        "n_experimentos_con_metricas": int(best_df["experiment"].nunique()) if not best_df.empty else 0,
        "n_figuras_generadas": int(len(figs)),
        "figuras": [f["archivo"] for f in figs],
    }
    safe_write(DATA_DIR / "manifest_generacion.json", json.dumps(manifest, indent=2, ensure_ascii=False))
    print(json.dumps(manifest, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
