#!/usr/bin/env python3
"""Genera ilustraciones específicas para el apartado 5.1 del TFM."""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_EXPERIMENTS_ROOT = PROJECT_ROOT / "experiments"
DEFAULT_OUTPUT_DIR = PROJECT_ROOT / "reports" / "tfm_figuras_cap5_1"
DEFAULT_MARGIN_RATIO = 0.05
DEFAULT_LOG_PATH = DEFAULT_OUTPUT_DIR / "LOG_regeneracion_curvas_loss_shared_ylim.md"
LOSS_EXPERIMENTS = (
    ("EXP1_SIMPLE_RGB", "5_2"),
    ("EXP2_SIMPLE_RGBD", "5_3"),
    ("EXP3_RESNET18_RGB_AUGMENT", "5_4"),
    ("EXP4_RESNET18_RGBD", "5_5"),
)
YLIM_POLICY_BY_EXPERIMENT: dict[str, str] = {}
MANUAL_SHARED_YLIM_BY_EXPERIMENT: dict[str, tuple[float, float]] = {
    "EXP2_SIMPLE_RGBD": (0.0300, 0.0400),
}
EDITORIAL_EXCEPTION_HINTS: dict[str, str] = {}


@dataclass(frozen=True)
class LossFigureReport:
    experiment: str
    figure_number: str
    output_path: Path
    seed_labels: tuple[str, ...]
    train_y_min: float
    train_y_max: float
    val_y_min: float
    val_y_max: float
    raw_min: float
    raw_max: float
    margin: float
    margin_ratio: float
    train_title: str
    val_title: str
    policy: str
    policy_note: str | None = None
    outlier_note: str | None = None


def _load_seed_metrics(exp_dir: Path) -> list[tuple[str, pd.DataFrame]]:
    seeds_data: list[tuple[str, pd.DataFrame]] = []
    required_columns = {"epoch", "train_loss", "val_loss"}

    for seed_dir in sorted(exp_dir.glob("seed_*")):
        metrics_csv = seed_dir / "metrics.csv"
        if not metrics_csv.exists():
            continue
        df = pd.read_csv(metrics_csv)
        missing = required_columns.difference(df.columns)
        if missing:
            raise ValueError(f"{metrics_csv} no contiene columnas requeridas: {sorted(missing)}")
        seed_num = seed_dir.name.replace("seed_", "")
        seeds_data.append((seed_num, df))

    if not seeds_data:
        raise FileNotFoundError(f"No se encontraron métricas válidas en {exp_dir}")

    return seeds_data


def _finite_values(series: pd.Series) -> list[float]:
    values = []
    for value in series.tolist():
        if pd.notna(value) and math.isfinite(float(value)):
            values.append(float(value))
    return values


def _detect_outlier_note(exp_name: str, values: list[float]) -> str | None:
    if len(values) < 4:
        return None

    series = pd.Series(values)
    q1 = float(series.quantile(0.25))
    q3 = float(series.quantile(0.75))
    iqr = q3 - q1
    if iqr <= 0:
        return None

    upper_fence = q3 + 3.0 * iqr
    max_value = max(values)
    if upper_fence <= 0 or max_value <= upper_fence or (max_value / upper_fence) <= 2.5:
        return None

    return (
        f"{exp_name}: se detectó un posible outlier alto en loss "
        f"(max={max_value:.6f}, umbral Tukey 3*IQR={upper_fence:.6f}). "
        "Se mantiene el rango completo sin recortar datos."
    )


def _compute_shared_ylim(seeds_data: list[tuple[str, pd.DataFrame]], margin_ratio: float) -> tuple[tuple[float, float], float, float, float, str | None]:
    all_values: list[float] = []
    for _, df in seeds_data:
        all_values.extend(_finite_values(df["train_loss"]))
        all_values.extend(_finite_values(df["val_loss"]))

    if not all_values:
        raise ValueError("No hay valores finitos de train_loss/val_loss para calcular el eje Y compartido.")

    raw_min = min(all_values)
    raw_max = max(all_values)
    span = raw_max - raw_min
    if span > 0:
        margin = span * margin_ratio
    else:
        reference_scale = max(abs(raw_min), abs(raw_max), 1e-3)
        margin = max(reference_scale * margin_ratio, 1e-4)
    return (raw_min - margin, raw_max + margin), raw_min, raw_max, margin, _detect_outlier_note("shared_ylim", all_values)


def _compute_series_ylim(
    seeds_data: list[tuple[str, pd.DataFrame]],
    column: str,
    margin_ratio: float,
) -> tuple[tuple[float, float], float, float, float]:
    values: list[float] = []
    for _, df in seeds_data:
        values.extend(_finite_values(df[column]))

    if not values:
        raise ValueError(f"No hay valores finitos de {column} para calcular el eje Y.")

    raw_min = min(values)
    raw_max = max(values)
    span = raw_max - raw_min
    if span > 0:
        margin = span * margin_ratio
    else:
        reference_scale = max(abs(raw_min), abs(raw_max), 1e-3)
        margin = max(reference_scale * margin_ratio, 1e-4)
    return (raw_min - margin, raw_max + margin), raw_min, raw_max, margin


def _resolve_ylim_policy(
    exp_name: str,
    seeds_data: list[tuple[str, pd.DataFrame]],
    margin_ratio: float,
) -> tuple[tuple[float, float], tuple[float, float], float, float, float, str, str | None, str | None]:
    policy = YLIM_POLICY_BY_EXPERIMENT.get(exp_name, "shared_global_train_val")
    if policy == "independent_train_val":
        train_ylim, train_raw_min, train_raw_max, train_margin = _compute_series_ylim(seeds_data, "train_loss", margin_ratio)
        val_ylim, val_raw_min, val_raw_max, val_margin = _compute_series_ylim(seeds_data, "val_loss", margin_ratio)
        policy_note = EDITORIAL_EXCEPTION_HINTS.get(exp_name)
        outlier_note = None
        return (
            train_ylim,
            val_ylim,
            min(train_raw_min, val_raw_min),
            max(train_raw_max, val_raw_max),
            max(train_margin, val_margin),
            policy,
            policy_note,
            outlier_note,
        )

    if policy != "shared_global_train_val":
        raise ValueError(f"Política de ylim no soportada para {exp_name}: {policy}")

    shared_ylim, raw_min, raw_max, margin, outlier_note = _compute_shared_ylim(seeds_data, margin_ratio)

    manual_ylim = MANUAL_SHARED_YLIM_BY_EXPERIMENT.get(exp_name)
    if manual_ylim is not None:
        if manual_ylim[0] > raw_min or manual_ylim[1] < raw_max:
            raise ValueError(
                f"El ylim manual de {exp_name} ({manual_ylim}) recorta informacion real "
                f"fuera del rango bruto [{raw_min:.6f}, {raw_max:.6f}]"
            )
        policy_note = (
            f"Se aplica un `ylim` manual compartido y documentado para {exp_name}: "
            f"[{manual_ylim[0]:.6f}, {manual_ylim[1]:.6f}]"
        )
        return manual_ylim, manual_ylim, raw_min, raw_max, margin, "manual_shared_ylim", policy_note, outlier_note

    return (
        shared_ylim,
        shared_ylim,
        raw_min,
        raw_max,
        margin,
        policy,
        EDITORIAL_EXCEPTION_HINTS.get(exp_name),
        outlier_note,
    )


def generar_curvas_loss_por_experimento(
    exp_name: str,
    exp_root: Path,
    out_dir: Path,
    fig_num: str,
    margin_ratio: float = DEFAULT_MARGIN_RATIO,
) -> LossFigureReport:
    """Genera curvas de train_loss y val_loss para un experimento específico."""
    exp_dir = exp_root / exp_name
    seeds_data = _load_seed_metrics(exp_dir)
    train_ylim, val_ylim, raw_min, raw_max, margin, policy, policy_note, outlier_note = _resolve_ylim_policy(
        exp_name,
        seeds_data,
        margin_ratio,
    )
    train_title = f"{exp_name} - Pérdida de Entrenamiento"
    val_title = f"{exp_name} - Pérdida de Validación"

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5), sharey=(train_ylim == val_ylim))

    for seed_num, df in seeds_data:
        ax1.plot(df["epoch"], df["train_loss"], marker="o", label=f"Seed {seed_num}", linewidth=2)
    ax1.set_xlabel("Época", fontsize=11)
    ax1.set_ylabel("Train Loss", fontsize=11)
    ax1.set_title(train_title, fontsize=12, fontweight="bold")
    ax1.grid(True, alpha=0.3)
    ax1.legend()
    ax1.set_ylim(*train_ylim)

    for seed_num, df in seeds_data:
        ax2.plot(df["epoch"], df["val_loss"], marker="s", label=f"Seed {seed_num}", linewidth=2)
    ax2.set_xlabel("Época", fontsize=11)
    ax2.set_ylabel("Val Loss", fontsize=11)
    ax2.set_title(val_title, fontsize=12, fontweight="bold")
    ax2.grid(True, alpha=0.3)
    ax2.legend()
    ax2.set_ylim(*val_ylim)

    plt.tight_layout()
    out_path = out_dir / f"ilustracion_{fig_num}_curvas_loss_{exp_name.lower()}.png"
    plt.savefig(out_path, dpi=200, bbox_inches="tight")
    plt.close()
    print(
        f"[OK] {out_path.name} generada con politica {policy} "
        f"(train=[{train_ylim[0]:.6f}, {train_ylim[1]:.6f}], "
        f"val=[{val_ylim[0]:.6f}, {val_ylim[1]:.6f}])"
    )

    return LossFigureReport(
        experiment=exp_name,
        figure_number=fig_num,
        output_path=out_path,
        seed_labels=tuple(f"Seed {seed_num}" for seed_num, _ in seeds_data),
        train_y_min=train_ylim[0],
        train_y_max=train_ylim[1],
        val_y_min=val_ylim[0],
        val_y_max=val_ylim[1],
        raw_min=raw_min,
        raw_max=raw_max,
        margin=margin,
        margin_ratio=margin_ratio,
        train_title=train_title,
        val_title=val_title,
        policy=policy,
        policy_note=policy_note,
        outlier_note=outlier_note.replace("shared_ylim", exp_name) if outlier_note else None,
    )


def generar_comparativa_por_seed_best_epoch(exp_root: Path, out_dir: Path, metric: str, fig_num: str, ylabel: str, title: str) -> None:
    """Genera gráfico de barras agrupadas por seed y experimento en best_epoch."""
    experiments = ["EXP1_SIMPLE_RGB", "EXP2_SIMPLE_RGBD", "EXP3_RESNET18_RGB_AUGMENT", "EXP4_RESNET18_RGBD"]
    
    data_by_exp = {}
    for exp in experiments:
        best_summary = exp_root / exp / "best_epoch_summary.csv"
        if not best_summary.exists():
            continue
        df = pd.read_csv(best_summary)
        data_by_exp[exp] = df[metric].values
    
    # Configurar posiciones de barras
    n_seeds = 3
    n_exps = len(data_by_exp)
    x = np.arange(n_exps)
    width = 0.25
    
    fig, ax = plt.subplots(figsize=(12, 6))
    
    for i in range(n_seeds):
        values = [data_by_exp[exp][i] if i < len(data_by_exp[exp]) else 0 for exp in experiments]
        ax.bar(x + i * width - width, values, width, label=f"Seed {i}", alpha=0.85)
    
    ax.set_xlabel("Experimento", fontsize=12, fontweight='bold')
    ax.set_ylabel(ylabel, fontsize=12, fontweight='bold')
    ax.set_title(title, fontsize=13, fontweight='bold')
    ax.set_xticks(x)
    ax.set_xticklabels([e.replace("_", "\n") for e in experiments], fontsize=9)
    ax.legend(fontsize=10)
    ax.grid(axis='y', alpha=0.3)
    
    plt.tight_layout()
    out_path = out_dir / f"ilustracion_{fig_num}_{metric}_por_seed_y_experimento.png"
    plt.savefig(out_path, dpi=200, bbox_inches='tight')
    plt.close()
    print(f"[OK] {out_path.name} generada")


def _write_loss_log(log_path: Path, reports: list[LossFigureReport]) -> None:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "# Regeneracion de curvas loss del capitulo 5",
        "",
        f"- Script modificado: `{Path(__file__).resolve()}`",
        "- Criterio de eje Y por defecto: min/max global conjunto de `train_loss` y `val_loss` por experimento, con margen simétrico del 5%, aplicado idénticamente a ambos subplots.",
        "- Soporte de excepciones: se permite configurar `ylim` manual compartido o `ylim` independiente por subplot si existe una justificación editorial explícita y sin recortar datos.",
        "- Nota del tutor incorporada: se fuerza en EXP2 un `ylim` manual compartido [0.0300, 0.0400] para facilitar comparación directa entre train y val.",
        "- Verificacion esperada: cada pareja train/val comparte exactamente el mismo `ylim`; se conservan títulos, leyendas y seeds.",
        "",
        "## Figuras regeneradas",
        "",
    ]

    for report in reports:
        lines.extend(
            [
                f"### {report.experiment}",
                "",
                f"- Figura fuente: `{report.output_path}`",
                f"- Seeds: {', '.join(report.seed_labels)}",
                f"- Titulo train: `{report.train_title}`",
                f"- Titulo val: `{report.val_title}`",
                f"- Politica aplicada: `{report.policy}`",
                f"- Rango bruto conjunto: [{report.raw_min:.6f}, {report.raw_max:.6f}]",
                f"- Margen aplicado: {report.margin:.6f} ({report.margin_ratio:.0%})",
                f"- `ylim` train: [{report.train_y_min:.6f}, {report.train_y_max:.6f}]",
                f"- `ylim` val: [{report.val_y_min:.6f}, {report.val_y_max:.6f}]",
                "",
            ]
        )
        if report.policy_note:
            lines.extend([f"- Nota editorial: {report.policy_note}", ""])
        if report.outlier_note:
            lines.extend([f"- Nota de outlier: {report.outlier_note}", ""])

    log_path.write_text("\n".join(lines), encoding="utf-8")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--experiments-root", type=Path, default=DEFAULT_EXPERIMENTS_ROOT)
    ap.add_argument("--out-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    ap.add_argument("--margin-ratio", type=float, default=DEFAULT_MARGIN_RATIO)
    ap.add_argument("--log-path", type=Path, default=DEFAULT_LOG_PATH)
    ap.add_argument(
        "--skip-best-epoch-comparisons",
        action="store_true",
        help="Regenera solo las curvas de loss (Ilustraciones 5-1 a 5-4).",
    )
    args = ap.parse_args()

    exp_root = args.experiments_root
    out_dir = args.out_dir
    out_dir.mkdir(parents=True, exist_ok=True)

    if not 0.03 <= args.margin_ratio <= 0.08:
        raise ValueError("--margin-ratio debe estar entre 0.03 y 0.08 para cumplir el criterio acordado.")

    loss_reports = [
        generar_curvas_loss_por_experimento(exp_name, exp_root, out_dir, fig_num, margin_ratio=args.margin_ratio)
        for exp_name, fig_num in LOSS_EXPERIMENTS
    ]
    _write_loss_log(args.log_path, loss_reports)

    if not args.skip_best_epoch_comparisons:
        generar_comparativa_por_seed_best_epoch(
            exp_root,
            out_dir,
            "val_success",
            "5_6",
            "Tasa de Éxito en Validación",
            "Comparativa de val_success por Seed y Experimento (Best Epoch)",
        )
        generar_comparativa_por_seed_best_epoch(
            exp_root,
            out_dir,
            "val_loss",
            "5_7",
            "Pérdida en Validación",
            "Comparativa de val_loss por Seed y Experimento (Best Epoch)",
        )

    print(f"\n[DONE] Curvas de loss regeneradas y log guardado en {args.log_path}")


if __name__ == "__main__":
    main()
