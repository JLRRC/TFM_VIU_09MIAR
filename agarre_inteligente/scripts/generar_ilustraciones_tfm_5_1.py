#!/usr/bin/env python3
"""Genera ilustraciones específicas para el apartado 5.1 del TFM."""

from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd
import numpy as np


def generar_curvas_loss_por_experimento(exp_name: str, exp_root: Path, out_dir: Path, fig_num: str) -> None:
    """Genera curvas de train_loss y val_loss para un experimento específico, separando seeds."""
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))
    
    exp_dir = exp_root / exp_name
    seeds_data = []
    
    for seed_dir in sorted(exp_dir.glob("seed_*")):
        metrics_csv = seed_dir / "metrics.csv"
        if not metrics_csv.exists():
            continue
        df = pd.read_csv(metrics_csv)
        seed_num = seed_dir.name.replace("seed_", "")
        seeds_data.append((seed_num, df))
    
    # Train loss
    for seed_num, df in seeds_data:
        ax1.plot(df["epoch"], df["train_loss"], marker='o', label=f"Seed {seed_num}", linewidth=2)
    ax1.set_xlabel("Época", fontsize=11)
    ax1.set_ylabel("Train Loss", fontsize=11)
    ax1.set_title(f"{exp_name} - Pérdida de Entrenamiento", fontsize=12, fontweight='bold')
    ax1.grid(True, alpha=0.3)
    ax1.legend()
    
    # Val loss
    for seed_num, df in seeds_data:
        ax2.plot(df["epoch"], df["val_loss"], marker='s', label=f"Seed {seed_num}", linewidth=2)
    ax2.set_xlabel("Época", fontsize=11)
    ax2.set_ylabel("Val Loss", fontsize=11)
    ax2.set_title(f"{exp_name} - Pérdida de Validación", fontsize=12, fontweight='bold')
    ax2.grid(True, alpha=0.3)
    ax2.legend()
    
    plt.tight_layout()
    out_path = out_dir / f"ilustracion_{fig_num}_curvas_loss_{exp_name.lower()}.png"
    plt.savefig(out_path, dpi=200, bbox_inches='tight')
    plt.close()
    print(f"[OK] {out_path.name} generada")


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


def main():
    exp_root = Path("experiments")
    out_dir = Path("reports/tfm_figuras_cap5_1")
    out_dir.mkdir(parents=True, exist_ok=True)
    
    # Ilustraciones 5-2 a 5-5: curvas de loss por experimento
    generar_curvas_loss_por_experimento("EXP1_SIMPLE_RGB", exp_root, out_dir, "5_2")
    generar_curvas_loss_por_experimento("EXP2_SIMPLE_RGBD", exp_root, out_dir, "5_3")
    generar_curvas_loss_por_experimento("EXP3_RESNET18_RGB_AUGMENT", exp_root, out_dir, "5_4")
    generar_curvas_loss_por_experimento("EXP4_RESNET18_RGBD", exp_root, out_dir, "5_5")
    
    # Ilustración 5-6: val_success por seed y experimento (best epoch)
    generar_comparativa_por_seed_best_epoch(
        exp_root, out_dir, "val_success", "5_6",
        "Tasa de Éxito en Validación",
        "Comparativa de val_success por Seed y Experimento (Best Epoch)"
    )
    
    # Ilustración 5-7: val_loss por seed y experimento (best epoch)
    generar_comparativa_por_seed_best_epoch(
        exp_root, out_dir, "val_loss", "5_7",
        "Pérdida en Validación",
        "Comparativa de val_loss por Seed y Experimento (Best Epoch)"
    )
    
    print("\n[DONE] Todas las figuras faltantes generadas")


if __name__ == "__main__":
    main()
