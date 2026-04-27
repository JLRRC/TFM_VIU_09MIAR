#!/usr/bin/env python3
"""Regenera artefactos oficiales del capitulo 5 tras reentrenar EXP1..EXP4."""

from __future__ import annotations

import argparse
import csv
import shutil
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


PROJECT_ROOT = Path(__file__).resolve().parents[2]
VISION_ROOT = PROJECT_ROOT / "agarre_inteligente"
EXPERIMENTS_ROOT = VISION_ROOT / "experiments"
REPORT_ROOT = PROJECT_ROOT / "report"

EXPERIMENT_ORDER = [
    "EXP1_SIMPLE_RGB",
    "EXP2_SIMPLE_RGBD",
    "EXP3_RESNET18_RGB_AUGMENT",
    "EXP4_RESNET18_RGBD",
]

EXPERIMENT_LABELS = {
    "EXP1_SIMPLE_RGB": "EXP1 RGB",
    "EXP2_SIMPLE_RGBD": "EXP2 RGB-D",
    "EXP3_RESNET18_RGB_AUGMENT": "EXP3 ResNet18 RGB+Aug",
    "EXP4_RESNET18_RGBD": "EXP4 ResNet18 RGB-D",
}

EXPERIMENT_COLORS = {
    "EXP1_SIMPLE_RGB": "#1f77b4",
    "EXP2_SIMPLE_RGBD": "#ff7f0e",
    "EXP3_RESNET18_RGB_AUGMENT": "#2ca02c",
    "EXP4_RESNET18_RGBD": "#d62728",
}

MODEL_MAP = {
    "EXP1_SIMPLE_RGB": "SimpleGraspCNN",
    "EXP2_SIMPLE_RGBD": "SimpleGraspCNN",
    "EXP3_RESNET18_RGB_AUGMENT": "ResNet18Grasp",
    "EXP4_RESNET18_RGBD": "ResNet18Grasp",
}

MODALITY_MAP = {
    "EXP1_SIMPLE_RGB": "RGB",
    "EXP2_SIMPLE_RGBD": "RGB-D",
    "EXP3_RESNET18_RGB_AUGMENT": "RGB",
    "EXP4_RESNET18_RGBD": "RGB-D",
}

CURVE_FIGURES = {
    "val_success": (
        "Ilustracion_5-5_evolucion_del_exito_de_agarre_en_validacion_por_epoca.png",
        "Exito de agarre en validacion",
        "Evolucion del exito de agarre en validacion por epoca",
    ),
    "val_iou": (
        "Ilustracion_5-6_evolucion_del_iou_medio_en_validacion_por_epoca.png",
        "IoU medio en validacion",
        "Evolucion del IoU medio en validacion por epoca",
    ),
    "val_angle_deg": (
        "Ilustracion_5-7_evolucion_del_error_angular_medio_en_validacion_por_epoca.png",
        "Error angular medio (Delta theta, grados)",
        "Evolucion del error angular medio en validacion por epoca",
    ),
}

BAR_COMPARISONS = {
    "val_success": (
        "Ilustracion_5-8_comparativa_de_val_success_por_seed_y_experimento_en_best_epoch.png",
        "Tasa de exito en validacion",
        "Comparativa de val_success por seed y experimento (best_epoch)",
    ),
    "val_loss": (
        "Ilustracion_5-9_comparativa_de_val_loss_por_seed_y_experimento_en_best_epoch.png",
        "Perdida en validacion",
        "Comparativa de val_loss por seed y experimento (best_epoch)",
    ),
}

FINAL_BARS = {
    "val_success_mean": (
        "Ilustracion_5-10_exito_final_de_agarre_en_validacion_agregado_por_experimento.png",
        "Exito final agregado",
    ),
    "val_iou_mean": (
        "Ilustracion_5-11_iou_medio_final_en_validacion_agregado_por_experimento.png",
        "IoU medio final agregado",
    ),
    "val_angle_mean_deg": (
        "Ilustracion_5-12_error_angular_medio_final_en_validacion_agregado_por_experimento.png",
        "Error angular medio final agregado",
    ),
}


@dataclass
class OutputSet:
    best_epoch_raw: list[Path]
    figures: list[Path]
    tables: list[Path]
    metrics: list[Path]
    backup_dir: Path
    trace_path: Path


def _backup_outputs(paths: list[Path], backup_root: Path) -> None:
    for path in paths:
        if not path.exists():
            continue
        target = backup_root / path.relative_to(PROJECT_ROOT)
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(path, target)


def _select_best_epoch(exp_dir: Path) -> pd.DataFrame:
    rows = []
    for metrics_csv in sorted(exp_dir.glob("seed_*/metrics.csv")):
        df = pd.read_csv(metrics_csv)
        if df.empty:
            continue
        best_idx = int(df["val_success"].idxmax())
        row = df.iloc[best_idx].to_dict()
        row["seed"] = int(metrics_csv.parent.name.replace("seed_", ""))
        row["best_epoch"] = int(row["epoch"])
        rows.append(row)
    out_df = pd.DataFrame(rows).sort_values("seed").reset_index(drop=True)
    out_path = exp_dir / "best_epoch_summary.csv"
    out_df.to_csv(out_path, index=False)
    return out_df


def _copy_best_epoch_summary(exp_name: str, best_df: pd.DataFrame) -> Path:
    out_path = REPORT_ROOT / "metrics" / "raw" / "experiments" / exp_name / "best_epoch_summary.csv"
    out_path.parent.mkdir(parents=True, exist_ok=True)
    best_df.to_csv(out_path, index=False)
    return out_path


def _build_aggregates(best_by_exp: dict[str, pd.DataFrame]) -> tuple[pd.DataFrame, pd.DataFrame]:
    seed_rows = []
    summary_rows = []

    for exp_name in EXPERIMENT_ORDER:
        df = best_by_exp[exp_name].copy()
        df.insert(0, "experiment", exp_name)
        seed_rows.append(df)

        summary_rows.append(
            {
                "experiment": exp_name,
                "n_seeds": int(len(df)),
                "best_epoch_mean": float(df["best_epoch"].mean()),
                "best_epoch_std": float(df["best_epoch"].std(ddof=0)),
                "val_success_mean": float(df["val_success"].mean()),
                "val_success_std": float(df["val_success"].std(ddof=0)),
                "val_iou_mean": float(df["val_iou"].mean()),
                "val_iou_std": float(df["val_iou"].std(ddof=0)),
                "val_angle_mean_deg": float(df["val_angle_deg"].mean()),
                "val_angle_std_deg": float(df["val_angle_deg"].std(ddof=0)),
                "val_loss_mean": float(df["val_loss"].mean()),
                "val_loss_std": float(df["val_loss"].std(ddof=0)),
                "source_best_epoch_summary": str(REPORT_ROOT / "metrics" / "raw" / "experiments" / exp_name / "best_epoch_summary.csv"),
            }
        )

    seed_df = pd.concat(seed_rows, ignore_index=True)
    summary_df = pd.DataFrame(summary_rows)
    return seed_df, summary_df


def _load_metric_series(exp_name: str, metric: str) -> tuple[pd.DataFrame, dict[str, int]]:
    frames = []
    seed_lengths: dict[str, int] = {}
    for metrics_csv in sorted((EXPERIMENTS_ROOT / exp_name).glob("seed_*/metrics.csv")):
        df = pd.read_csv(metrics_csv)[["epoch", metric]].copy().sort_values("epoch")
        seed_name = metrics_csv.parent.name
        seed_lengths[seed_name] = int(df["epoch"].max())
        df["seed"] = seed_name
        frames.append(df)
    all_df = pd.concat(frames, ignore_index=True)
    summary = all_df.groupby("epoch")[metric].agg(["mean", "std"]).reset_index()
    summary["std"] = summary["std"].fillna(0.0)
    return summary, seed_lengths


def _plot_metric_curves() -> list[Path]:
    outputs: list[Path] = []
    fig_dir = REPORT_ROOT / "figures" / "cap5"
    fig_dir.mkdir(parents=True, exist_ok=True)

    plt.rcParams.update(
        {
            "figure.dpi": 100,
            "savefig.dpi": 300,
            "font.size": 10,
            "axes.labelsize": 11,
            "axes.titlesize": 12,
            "legend.fontsize": 9,
        }
    )

    for metric, (filename, ylabel, title) in CURVE_FIGURES.items():
        fig, ax = plt.subplots(figsize=(9, 5.2))
        for exp_name in EXPERIMENT_ORDER:
            summary, _ = _load_metric_series(exp_name, metric)
            x = summary["epoch"].to_numpy(dtype=float)
            y = summary["mean"].to_numpy(dtype=float)
            s = summary["std"].to_numpy(dtype=float)
            color = EXPERIMENT_COLORS[exp_name]
            ax.plot(x, y, color=color, linewidth=2.1, label=EXPERIMENT_LABELS[exp_name])
            ax.fill_between(x, y - s, y + s, color=color, alpha=0.18, linewidth=0)

        ax.set_title(title)
        ax.set_xlabel("Epoca")
        ax.set_ylabel(ylabel)
        ax.grid(True, linestyle="--", alpha=0.35)
        ax.legend(loc="best", frameon=True)
        ax.set_xlim(left=1)
        fig.tight_layout()
        out_path = fig_dir / filename
        fig.savefig(out_path)
        plt.close(fig)
        outputs.append(out_path)

    return outputs


def _grouped_bar(best_by_exp: dict[str, pd.DataFrame], metric: str) -> Path:
    filename, ylabel, title = BAR_COMPARISONS[metric]
    fig_dir = REPORT_ROOT / "figures" / "cap5"
    fig, ax = plt.subplots(figsize=(12, 6))
    x = np.arange(len(EXPERIMENT_ORDER))
    width = 0.25
    for seed in range(3):
        vals = []
        for exp_name in EXPERIMENT_ORDER:
            df = best_by_exp[exp_name]
            row = df[df["seed"] == seed]
            vals.append(float(row.iloc[0][metric]))
        ax.bar(x + seed * width - width, vals, width, label=f"Seed {seed}", alpha=0.85)

    ax.set_xlabel("Experimento", fontsize=12, fontweight="bold")
    ax.set_ylabel(ylabel, fontsize=12, fontweight="bold")
    ax.set_title(title, fontsize=13, fontweight="bold")
    ax.set_xticks(x)
    ax.set_xticklabels([exp.replace("_", "\n") for exp in EXPERIMENT_ORDER], fontsize=9)
    ax.legend(fontsize=10)
    ax.grid(axis="y", alpha=0.3)
    fig.tight_layout()
    out_path = fig_dir / filename
    fig.savefig(out_path, dpi=200, bbox_inches="tight")
    plt.close(fig)
    return out_path


def _final_bar(summary_df: pd.DataFrame, metric: str) -> Path:
    filename, title = FINAL_BARS[metric]
    fig_dir = REPORT_ROOT / "figures" / "cap5"
    fig, ax = plt.subplots(figsize=(8, 5))
    colors = [EXPERIMENT_COLORS[exp] for exp in summary_df["experiment"]]
    ax.bar(summary_df["experiment"], summary_df[metric], color=colors)
    ax.set_title(title)
    ax.grid(axis="y", alpha=0.3)
    ax.tick_params(axis="x", rotation=25)
    fig.tight_layout()
    out_path = fig_dir / filename
    fig.savefig(out_path, dpi=200)
    plt.close(fig)
    return out_path


def _write_tables(summary_df: pd.DataFrame) -> list[Path]:
    out_dir = REPORT_ROOT / "tables" / "cap5"
    out_dir.mkdir(parents=True, exist_ok=True)
    annex_dir = REPORT_ROOT / "tables" / "anexos"
    annex_dir.mkdir(parents=True, exist_ok=True)

    summary = summary_df.copy()
    summary["model"] = summary["experiment"].map(MODEL_MAP)
    summary["modality"] = summary["experiment"].map(MODALITY_MAP)

    table_5_1 = out_dir / "Tabla_5-1_resultados_agregados_en_validacion_best_epoch_por_ejecucion_bajo_split_object_wi.csv"
    summary.rename(
        columns={
            "val_angle_mean_deg": "val_angle_deg_mean",
            "val_angle_std_deg": "val_angle_deg_std",
        }
    )[
        [
            "experiment",
            "n_seeds",
            "val_success_mean",
            "val_success_std",
            "val_iou_mean",
            "val_iou_std",
            "val_angle_deg_mean",
            "val_angle_deg_std",
            "val_loss_mean",
            "val_loss_std",
            "model",
            "modality",
        ]
    ].to_csv(table_5_1, index=False)

    compact = pd.DataFrame(
        {
            "experiment": summary["experiment"],
            "model": summary["model"],
            "modality": summary["modality"],
            "val_success": [f"{a:.4f} +- {b:.4f}" for a, b in zip(summary["val_success_mean"], summary["val_success_std"])],
            "val_iou": [f"{a:.4f} +- {b:.4f}" for a, b in zip(summary["val_iou_mean"], summary["val_iou_std"])],
            "val_angle_deg": [f"{a:.4f} +- {b:.4f}" for a, b in zip(summary["val_angle_mean_deg"], summary["val_angle_std_deg"])],
            "val_loss": [f"{a:.4f} +- {b:.4f}" for a, b in zip(summary["val_loss_mean"], summary["val_loss_std"])],
        }
    )
    table_5_2 = out_dir / "Tabla_5-2_resumen_de_metricas_finales_por_experimento_en_validacion.csv"
    compact.to_csv(table_5_2, index=False)

    rows = []
    for modality in ["RGB", "RGB-D"]:
        resnet = summary[(summary["model"] == "ResNet18Grasp") & (summary["modality"] == modality)].iloc[0]
        simple = summary[(summary["model"] == "SimpleGraspCNN") & (summary["modality"] == modality)].iloc[0]
        rows.append(
            {
                "modality": modality,
                "resnet18_val_success": float(resnet["val_success_mean"]),
                "simplecnn_val_success": float(simple["val_success_mean"]),
                "delta_abs_points": float(resnet["val_success_mean"] - simple["val_success_mean"]),
            }
        )
    table_5_4 = out_dir / "Tabla_5-4_comparativa_por_modalidad_entre_simplegraspcnn_y_resnet18grasp.csv"
    modality_df = pd.DataFrame(rows)
    modality_df.to_csv(table_5_4, index=False)

    table_8_1 = annex_dir / "Tabla_8-1_resultados_por_semilla_y_experimento_en_la_mejor_epoca_de_validacion.csv"
    pd.read_csv(REPORT_ROOT / "metrics" / "aggregated" / "chapter5_best_epoch_runs_all_seeds.csv").to_csv(table_8_1, index=False)

    table_8_3 = annex_dir / "Tabla_8-3_resumen_de_experimentos_base_en_validacion_media_desviacion_estandar_cuando_proc.csv"
    summary.rename(
        columns={
            "val_angle_mean_deg": "val_angle_deg_mean",
            "val_angle_std_deg": "val_angle_deg_std",
        }
    )[
        [
            "experiment",
            "n_seeds",
            "val_success_mean",
            "val_success_std",
            "val_iou_mean",
            "val_iou_std",
            "val_angle_deg_mean",
            "val_angle_deg_std",
            "val_loss_mean",
            "val_loss_std",
        ]
    ].to_csv(table_8_3, index=False)

    table_8_4 = annex_dir / "Tabla_8-4_comparativa_por_modalidad_entre_simplegraspcnn_y_resnet18grasp_mejor_epoca_de_va.csv"
    modality_df.to_csv(table_8_4, index=False)

    return [table_5_1, table_5_2, table_5_4, table_8_1, table_8_3, table_8_4]


def _write_trace(
    trace_path: Path,
    backup_dir: Path,
    seed_df: pd.DataFrame,
    summary_df: pd.DataFrame,
    outputs: OutputSet,
) -> None:
    trace_path.parent.mkdir(parents=True, exist_ok=True)
    with trace_path.open("w", encoding="utf-8", newline="") as fh:
        writer = csv.writer(fh, delimiter="\t")
        writer.writerow(["timestamp", "event", "detail"])
        writer.writerow([datetime.now().isoformat(timespec="seconds"), "backup_dir", str(backup_dir)])
        writer.writerow([datetime.now().isoformat(timespec="seconds"), "script", str(Path(__file__).resolve())])
        writer.writerow([datetime.now().isoformat(timespec="seconds"), "seed_rows", str(len(seed_df))])
        writer.writerow([datetime.now().isoformat(timespec="seconds"), "summary_rows", str(len(summary_df))])
        for path in outputs.best_epoch_raw + outputs.metrics + outputs.tables + outputs.figures:
            writer.writerow([datetime.now().isoformat(timespec="seconds"), "generated", str(path)])
        for row in summary_df.itertuples(index=False):
            writer.writerow(
                [
                    datetime.now().isoformat(timespec="seconds"),
                    "summary",
                    f"{row.experiment}: success_mean={row.val_success_mean:.6f}, iou_mean={row.val_iou_mean:.6f}, angle_mean={row.val_angle_mean_deg:.6f}, val_loss_mean={row.val_loss_mean:.6f}",
                ]
            )


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--stamp", default=datetime.now().strftime("%Y%m%d_%H%M%S"))
    args = ap.parse_args()

    stamp = args.stamp
    backup_dir = PROJECT_ROOT / "BORRAR" / "report_extra" / f"chapter5_pre_retrain_{stamp}"
    trace_path = REPORT_ROOT / "logs" / "reproducibility" / f"chapter5_post_retrain_regeneration_{stamp}.tsv"

    raw_targets = [REPORT_ROOT / "metrics" / "raw" / "experiments" / exp / "best_epoch_summary.csv" for exp in EXPERIMENT_ORDER]
    metric_targets = [
        REPORT_ROOT / "metrics" / "aggregated" / "chapter5_best_epoch_runs_all_seeds.csv",
        REPORT_ROOT / "metrics" / "validated" / "chapter5_experiment_summary_validated.csv",
    ]
    figure_targets = [REPORT_ROOT / "figures" / "cap5" / spec[0] for spec in CURVE_FIGURES.values()]
    figure_targets += [REPORT_ROOT / "figures" / "cap5" / spec[0] for spec in BAR_COMPARISONS.values()]
    figure_targets += [REPORT_ROOT / "figures" / "cap5" / spec[0] for spec in FINAL_BARS.values()]
    table_targets = [
        REPORT_ROOT / "tables" / "cap5" / "Tabla_5-1_resultados_agregados_en_validacion_best_epoch_por_ejecucion_bajo_split_object_wi.csv",
        REPORT_ROOT / "tables" / "cap5" / "Tabla_5-2_resumen_de_metricas_finales_por_experimento_en_validacion.csv",
        REPORT_ROOT / "tables" / "cap5" / "Tabla_5-4_comparativa_por_modalidad_entre_simplegraspcnn_y_resnet18grasp.csv",
        REPORT_ROOT / "tables" / "anexos" / "Tabla_8-1_resultados_por_semilla_y_experimento_en_la_mejor_epoca_de_validacion.csv",
        REPORT_ROOT / "tables" / "anexos" / "Tabla_8-3_resumen_de_experimentos_base_en_validacion_media_desviacion_estandar_cuando_proc.csv",
        REPORT_ROOT / "tables" / "anexos" / "Tabla_8-4_comparativa_por_modalidad_entre_simplegraspcnn_y_resnet18grasp_mejor_epoca_de_va.csv",
    ]

    _backup_outputs(raw_targets + metric_targets + figure_targets + table_targets, backup_dir)

    best_by_exp: dict[str, pd.DataFrame] = {}
    raw_outputs: list[Path] = []
    for exp_name in EXPERIMENT_ORDER:
        best_df = _select_best_epoch(EXPERIMENTS_ROOT / exp_name)
        best_by_exp[exp_name] = best_df
        raw_outputs.append(_copy_best_epoch_summary(exp_name, best_df))

    seed_df, summary_df = _build_aggregates(best_by_exp)

    metric_paths = [
        REPORT_ROOT / "metrics" / "aggregated" / "chapter5_best_epoch_runs_all_seeds.csv",
        REPORT_ROOT / "metrics" / "validated" / "chapter5_experiment_summary_validated.csv",
    ]
    metric_paths[0].parent.mkdir(parents=True, exist_ok=True)
    metric_paths[1].parent.mkdir(parents=True, exist_ok=True)
    seed_df.to_csv(metric_paths[0], index=False)
    summary_df.to_csv(metric_paths[1], index=False)

    figures = _plot_metric_curves()
    figures.extend(_grouped_bar(best_by_exp, metric) for metric in BAR_COMPARISONS)
    figures.extend(_final_bar(summary_df, metric) for metric in FINAL_BARS)

    tables = _write_tables(summary_df)

    outputs = OutputSet(
        best_epoch_raw=raw_outputs,
        figures=figures,
        tables=tables,
        metrics=metric_paths,
        backup_dir=backup_dir,
        trace_path=trace_path,
    )
    _write_trace(trace_path, backup_dir, seed_df, summary_df, outputs)

    print(f"[OK] Regeneracion post-retrain completada. Traza: {trace_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
