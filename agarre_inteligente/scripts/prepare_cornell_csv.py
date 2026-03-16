#!/usr/bin/env python3
"""
Script para procesar el Cornell Grasping Dataset y generar CSVs
con split object-wise para train/val.

Formato de entrada (Cornell):
- Imágenes RGB: pcd<NUM>r.png
- Imágenes depth: pcd<NUM>d.png  
- Anotaciones: pcd<NUM>cpos.txt (4 líneas con coordenadas x,y de rectángulo)

Formato de salida:
- CSVs con columnas: image_path,depth_path,cx,cy,w,h,angle_deg
- Split object-wise: 70% train, 30% val (por directorio de objeto)
"""

import argparse
import numpy as np
import pandas as pd
from pathlib import Path
import sys
from typing import List, Tuple


def parse_cornell_annotation(txt_path: Path) -> List[Tuple[float, float, float, float, float]]:
    """
    Parsea archivo de anotación Cornell (4 puntos del rectángulo).
    
    Formato de archivo:
        x1 y1
        x2 y2
        x3 y3
        x4 y4
        
    Puede haber múltiples rectángulos (grupos de 4 líneas).
    
    Returns:
        Lista de tuplas (cx, cy, w, h, angle_deg) para cada rectángulo.
    """
    if not txt_path.exists():
        return []
    
    with open(txt_path, 'r') as f:
        lines = [line.strip() for line in f if line.strip()]
    
    # Cornell tiene múltiples anotaciones por imagen (grupos de 4 líneas)
    rectangles = []
    for i in range(0, len(lines), 4):
        if i + 3 >= len(lines):
            break
            
        try:
            # Leer 4 puntos
            points = []
            for j in range(4):
                x, y = map(float, lines[i + j].split())
                points.append([x, y])
            points = np.array(points)
            
            # Calcular centro
            cx = points[:, 0].mean()
            cy = points[:, 1].mean()
            
            # Calcular dimensiones (ancho y alto del rectángulo orientado)
            # Usar distancias entre puntos consecutivos
            d01 = np.linalg.norm(points[1] - points[0])
            d12 = np.linalg.norm(points[2] - points[1])
            
            # El lado más largo es el ancho (width), el corto es alto (height)
            if d01 > d12:
                w, h = d01, d12
                # Ángulo basado en edge 0->1
                dx = points[1, 0] - points[0, 0]
                dy = points[1, 1] - points[0, 1]
            else:
                w, h = d12, d01
                # Ángulo basado en edge 1->2
                dx = points[2, 0] - points[1, 0]
                dy = points[2, 1] - points[1, 1]
            
            # Calcular ángulo en grados [-90, 90]
            angle_rad = np.arctan2(dy, dx)
            angle_deg = np.degrees(angle_rad)
            
            # Normalizar a rango [-90, 90] (simetría de agarre)
            while angle_deg > 90:
                angle_deg -= 180
            while angle_deg < -90:
                angle_deg += 180
            
            rectangles.append((cx, cy, w, h, angle_deg))
            
        except (ValueError, IndexError) as e:
            print(f"[WARN] Error parseando {txt_path}: {e}")
            continue
    
    return rectangles


def process_cornell_dataset(raw_dir: Path, val_split: float = 0.3, seed: int = 42):
    """
    Procesa el dataset Cornell completo.
    
    Args:
        raw_dir: Directorio con estructura Cornell (carpetas 01, 02, ...)
        val_split: Proporción para validación (object-wise)
        seed: Semilla para reproducibilidad
        
    Returns:
        train_df, val_df: DataFrames con columnas (image_path, depth_path, cx, cy, w, h, angle_deg)
    """
    np.random.seed(seed)
    
    # Buscar directorios de objetos (01, 02, 03, ...)
    object_dirs = sorted([d for d in raw_dir.iterdir() if d.is_dir() and d.name.isdigit()])
    
    if len(object_dirs) == 0:
        print(f"[ERROR] No se encontraron directorios de objetos en {raw_dir}")
        sys.exit(1)
    
    print(f"[OK] Encontrados {len(object_dirs)} objetos")
    
    # Split object-wise
    np.random.shuffle(object_dirs)
    n_val = max(1, int(len(object_dirs) * val_split))
    val_objects = object_dirs[:n_val]
    train_objects = object_dirs[n_val:]
    
    print(f"[OK] Split: {len(train_objects)} objetos train, {len(val_objects)} objetos val")
    
    # Procesar cada conjunto
    train_rows = []
    val_rows = []
    
    for obj_dir in train_objects:
        rows = _process_object_dir(obj_dir, raw_dir)
        train_rows.extend(rows)
    
    for obj_dir in val_objects:
        rows = _process_object_dir(obj_dir, raw_dir)
        val_rows.extend(rows)
    
    train_df = pd.DataFrame(train_rows, columns=['image_path', 'depth_path', 'cx', 'cy', 'w', 'h', 'angle_deg'])
    val_df = pd.DataFrame(val_rows, columns=['image_path', 'depth_path', 'cx', 'cy', 'w', 'h', 'angle_deg'])
    
    print(f"[OK] Train samples: {len(train_df)}")
    print(f"[OK] Val samples: {len(val_df)}")
    
    return train_df, val_df


def _process_object_dir(obj_dir: Path, raw_dir: Path) -> List[List]:
    """Procesa un directorio de objeto (e.g., 01/) y retorna filas CSV."""
    rows = []
    
    # Buscar imágenes RGB
    rgb_files = sorted(obj_dir.glob("pcd*r.png"))
    
    for rgb_path in rgb_files:
        # Construir rutas correspondientes
        base_name = rgb_path.stem[:-1]  # Quitar 'r' final
        # Cornell real suele usar profundidad en .tiff
        depth_candidates = [
            rgb_path.parent / f"{base_name}d.png",
            rgb_path.parent / f"{base_name}d.tiff",
            rgb_path.parent / f"{base_name}d.tif",
        ]
        depth_path = next((p for p in depth_candidates if p.exists()), None)
        annot_path = rgb_path.parent / f"{base_name}cpos.txt"
        
        if depth_path is None:
            print(f"[WARN] Depth no encontrado para base: {base_name}")
            continue
        
        if not annot_path.exists():
            print(f"[WARN] Anotación no encontrada: {annot_path}")
            continue
        
        # Parsear anotaciones
        rectangles = parse_cornell_annotation(annot_path)
        
        if len(rectangles) == 0:
            print(f"[WARN] Sin rectángulos válidos: {annot_path}")
            continue
        
        # Crear una fila por cada rectángulo anotado
        for cx, cy, w, h, angle_deg in rectangles:
            # Rutas relativas desde directorio raíz del proyecto (data/raw/cornell -> data/raw/cornell)
            # Asumiendo que data_root en config será "." o ruta desde raíz proyecto
            rel_rgb = rgb_path
            rel_depth = depth_path
            
            rows.append([
                str(rel_rgb),
                str(rel_depth),
                cx, cy, w, h, angle_deg
            ])
    
    return rows


def main():
    parser = argparse.ArgumentParser(description="Preparar Cornell Grasping Dataset")
    parser.add_argument("--raw-dir", type=str, required=True,
                        help="Directorio raw con estructura Cornell")
    parser.add_argument("--out-dir", type=str, required=True,
                        help="Directorio de salida para CSVs")
    parser.add_argument("--val-split", type=float, default=0.3,
                        help="Proporción para validación (object-wise)")
    parser.add_argument("--seed", type=int, default=42,
                        help="Semilla para split reproducible")
    
    args = parser.parse_args()
    
    raw_dir = Path(args.raw_dir)
    out_dir = Path(args.out_dir)
    
    if not raw_dir.exists():
        print(f"[ERROR] Directorio no existe: {raw_dir}")
        sys.exit(1)
    
    print("=" * 50)
    print("  Preparando Cornell Grasping Dataset")
    print("=" * 50)
    print(f"Raw dir:   {raw_dir}")
    print(f"Out dir:   {out_dir}")
    print(f"Val split: {args.val_split}")
    print(f"Seed:      {args.seed}")
    print()
    
    # Procesar dataset
    train_df, val_df = process_cornell_dataset(raw_dir, args.val_split, args.seed)
    
    # Crear directorio de salida
    splits_dir = out_dir / "splits" / "object_wise"
    splits_dir.mkdir(parents=True, exist_ok=True)
    
    # Guardar CSVs
    train_csv = splits_dir / "train.csv"
    val_csv = splits_dir / "val.csv"
    
    train_df.to_csv(train_csv, index=False)
    val_df.to_csv(val_csv, index=False)
    
    print(f"[OK] Train CSV guardado: {train_csv}")
    print(f"[OK] Val CSV guardado: {val_csv}")
    print()
    
    # Estadísticas
    print("=" * 50)
    print("  Estadísticas")
    print("=" * 50)
    print(f"Train samples: {len(train_df)}")
    print(f"Val samples:   {len(val_df)}")
    print(f"Total:         {len(train_df) + len(val_df)}")
    print()
    
    print("Distribución de ángulos (train):")
    print(train_df['angle_deg'].describe())
    print()
    
    print("Distribución de dimensiones (train):")
    print(train_df[['w', 'h']].describe())
    print()
    
    print("[OK] Preparación completada")
    
    # Actualizar archivos de índices limpios
    audit_dir = Path("reports/cornell_audit")
    if audit_dir.exists():
        # Guardar índices (para trazabilidad)
        with open(audit_dir / "clean_idx_train_v2.txt", "w") as f:
            f.write(f"# Índices train: N={len(train_df)}\n")
            f.write("# Generado automáticamente por prepare_cornell_csv.py\n")
        
        with open(audit_dir / "clean_idx_val.txt", "w") as f:
            f.write(f"# Índices val: N={len(val_df)}\n")
            f.write("# Generado automáticamente por prepare_cornell_csv.py\n")
        
        print(f"[OK] Índices actualizados en {audit_dir}")


if __name__ == "__main__":
    main()
