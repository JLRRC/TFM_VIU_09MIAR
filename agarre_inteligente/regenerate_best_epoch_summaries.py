#!/usr/bin/env python3
"""Regenera best_epoch_summary.csv para cada experimento desde los metrics.csv actualizados."""

from pathlib import Path
import pandas as pd


def regenerate_best_epoch_summary(exp_dir: Path):
    """Lee metrics.csv de cada seed y genera best_epoch_summary.csv."""
    rows = []
    
    for seed_dir in sorted(exp_dir.glob("seed_*")):
        metrics_csv = seed_dir / "metrics.csv"
        if not metrics_csv.exists():
            continue
        
        df = pd.read_csv(metrics_csv)
        if df.empty:
            continue
        
        # Tomar última época (mejor modelo guardado)
        best_row = df.iloc[-1].copy()
        
        # Agregar columna seed
        seed_num = int(seed_dir.name.split("_")[1])
        best_row["seed"] = seed_num
        best_row["best_epoch"] = int(best_row["epoch"])
        
        rows.append(best_row)
    
    if not rows:
        return
    
    # Guardar resumen
    summary_df = pd.DataFrame(rows)
    summary_csv = exp_dir / "best_epoch_summary.csv"
    summary_df.to_csv(summary_csv, index=False)
    
    print(f"✅ {exp_dir.name}: Regenerado con {len(rows)} seeds")
    print(f"   Success rates: {summary_df['val_success'].values}")


def main():
    experiments_root = Path("experiments")
    
    for exp_dir in sorted(experiments_root.glob("EXP*")):
        if not exp_dir.is_dir():
            continue
        regenerate_best_epoch_summary(exp_dir)
    
    print("\n[OK] Todos los best_epoch_summary.csv regenerados")


if __name__ == "__main__":
    main()
