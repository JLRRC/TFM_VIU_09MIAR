#!/usr/bin/env python3
"""Regenera datos y figuras del workspace del articulo sin tocar borradores editoriales."""

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

GEN_MARK = "<!-- generado-articulo -->"


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
        "salidas": "Vector de 5 parametros: cx, cy, w, h, angle. En dataset se entrenan normalizados.",
        "uso_real": "Entrenamiento oficial EXP1/EXP2 e inferencia CLI generica.",
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
        "uso_real": "Entrenamiento oficial EXP3/EXP4 e inferencia CLI generica.",
        "estado": "Activo y modelo principal del bloque oficial.",
        "evidencia": "factory.py construye ResNetGrasp para ResNet18Grasp; configs EXP3/EXP4; metrics en experiments/EXP3 y EXP4.",
    },
    {
        "nombre_codigo": "SimpleGrasp",
        "clase_real": "SimpleGrasp",
        "archivo_definicion": "agarre_inteligente/src/models/simple_grasp.py",
        "arquitectura": "CNN ligera alineada con diseno teorico: primera Conv2d 7x7 stride 2, bloques 3x3, AdaptiveAvgPool2d(7,7), MLP 128*7*7-256-5.",
        "entradas": "Tensor imagen 224x224 RGB o RGB-D segun input_channels.",
        "salidas": "Vector de 5 parametros normalizados.",
        "uso_real": "Experimentos auxiliares EXP1.1/EXP1.2.",
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
        "uso_real": "Referencia legacy no usada en el argumento experimental principal.",
        "estado": "Legacy/obsoleto o referencia rota; no debe presentarse como modelo implementado actual.",
        "evidencia": "No se localiza un modulo actual `graspnet.models.simple_grasp_cnn` en el arbol inspeccionado.",
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
    """Los Markdown publicables se editan manualmente; no regenerarlos desde plantillas obsoletas."""
    _ = (exp_df, best_df, summary, figs)
    return


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
    (DATA_DIR / "manifest_generacion.json").write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(manifest, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
