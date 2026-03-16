#!/usr/bin/env python3
"""
Generador de dataset sintético en formato Cornell para pruebas.
Crea imágenes RGB, depth y anotaciones que simulan la estructura real.
"""

import argparse
import numpy as np
from pathlib import Path
from PIL import Image, ImageDraw
import pandas as pd
import sys


def generate_synthetic_object(obj_id: int, n_images: int, out_dir: Path, seed: int = 42):
    """
    Genera un "objeto" sintético con múltiples vistas.
    
    Args:
        obj_id: ID del objeto (01, 02, ...)
        n_images: Número de imágenes por objeto
        out_dir: Directorio de salida
        seed: Semilla para reproducibilidad
    """
    np.random.seed(seed + obj_id)
    
    obj_dir = out_dir / f"{obj_id:02d}"
    obj_dir.mkdir(parents=True, exist_ok=True)
    
    # Color base del objeto (por objeto)
    base_color = tuple(np.random.randint(50, 200, 3).tolist())
    
    for img_id in range(n_images):
        img_num = obj_id * 100 + img_id
        
        # Generar imagen RGB (224x224)
        img_rgb = generate_synthetic_rgb(base_color, seed + img_num)
        rgb_path = obj_dir / f"pcd{img_num:04d}r.png"
        Image.fromarray(img_rgb).save(rgb_path)
        
        # Generar mapa de profundidad (16-bit)
        img_depth = generate_synthetic_depth(seed + img_num)
        depth_path = obj_dir / f"pcd{img_num:04d}d.png"
        Image.fromarray(img_depth).save(depth_path)
        
        # Generar anotaciones (2-4 grasps por imagen)
        n_grasps = np.random.randint(2, 5)
        annot_path = obj_dir / f"pcd{img_num:04d}cpos.txt"
        generate_cornell_annotation(annot_path, n_grasps, seed + img_num)


def generate_synthetic_rgb(base_color: tuple, seed: int) -> np.ndarray:
    """Genera imagen RGB sintética con forma simplificada."""
    np.random.seed(seed)
    
    img = np.ones((224, 224, 3), dtype=np.uint8) * 200  # Fondo gris claro
    
    # Agregar "objeto" como forma geométrica
    shape_type = np.random.choice(['circle', 'rectangle', 'polygon'])
    
    img_pil = Image.fromarray(img)
    draw = ImageDraw.Draw(img_pil)
    
    cx, cy = np.random.randint(60, 164, 2)
    size = np.random.randint(30, 80)
    
    if shape_type == 'circle':
        draw.ellipse([cx-size, cy-size, cx+size, cy+size], fill=base_color)
    elif shape_type == 'rectangle':
        angle = np.random.uniform(-45, 45)
        # Simplificado: rectángulo axis-aligned
        w, h = np.random.randint(40, 100, 2)
        draw.rectangle([cx-w//2, cy-h//2, cx+w//2, cy+h//2], fill=base_color)
    else:  # polygon
        n_sides = np.random.randint(5, 8)
        angles = np.linspace(0, 2*np.pi, n_sides, endpoint=False)
        radius = size
        points = [(cx + radius*np.cos(a), cy + radius*np.sin(a)) for a in angles]
        draw.polygon(points, fill=base_color)
    
    # Agregar ruido
    noise = np.random.randint(-20, 20, (224, 224, 3))
    img_final = np.array(img_pil).astype(np.int16) + noise
    img_final = np.clip(img_final, 0, 255).astype(np.uint8)
    
    return img_final


def generate_synthetic_depth(seed: int) -> np.ndarray:
    """Genera mapa de profundidad sintético (16-bit)."""
    np.random.seed(seed)
    
    # Profundidad base (500-1500 mm)
    base_depth = np.random.randint(700, 1200)
    depth = np.ones((224, 224), dtype=np.uint16) * base_depth
    
    # Agregar objeto más cercano
    cx, cy = np.random.randint(60, 164, 2)
    size = np.random.randint(30, 80)
    
    y, x = np.ogrid[:224, :224]
    mask = (x - cx)**2 + (y - cy)**2 <= size**2
    depth[mask] = base_depth - np.random.randint(100, 300)
    
    # Agregar ruido
    noise = np.random.randint(-50, 50, (224, 224))
    depth = (depth.astype(np.int32) + noise).clip(0, 65535).astype(np.uint16)
    
    return depth


def generate_cornell_annotation(annot_path: Path, n_grasps: int, seed: int):
    """
    Genera anotación en formato Cornell (4 puntos por grasp).
    
    Formato:
        x1 y1
        x2 y2
        x3 y3
        x4 y4
        (repetir por cada grasp)
    """
    np.random.seed(seed)
    
    lines = []
    for _ in range(n_grasps):
        # Centro del grasp
        cx = np.random.uniform(60, 164)
        cy = np.random.uniform(60, 164)
        
        # Dimensiones
        w = np.random.uniform(40, 100)
        h = np.random.uniform(15, 40)
        
        # Ángulo
        angle = np.random.uniform(-np.pi/2, np.pi/2)
        
        # Calcular 4 puntos del rectángulo orientado
        cos_a = np.cos(angle)
        sin_a = np.sin(angle)
        
        # Esquinas en sistema local
        corners_local = np.array([
            [-w/2, -h/2],
            [w/2, -h/2],
            [w/2, h/2],
            [-w/2, h/2]
        ])
        
        # Rotar y trasladar
        rotation = np.array([[cos_a, -sin_a], [sin_a, cos_a]])
        corners_global = corners_local @ rotation.T + np.array([cx, cy])
        
        # Escribir en formato Cornell
        for corner in corners_global:
            lines.append(f"{corner[0]:.2f} {corner[1]:.2f}")
    
    annot_path.write_text("\n".join(lines) + "\n")


def generate_full_dataset(out_dir: Path, n_objects: int = 50, imgs_per_object: int = 5, seed: int = 42):
    """
    Genera dataset completo sintético.
    
    Args:
        out_dir: Directorio de salida (e.g., data/raw/cornell_synthetic)
        n_objects: Número de objetos diferentes
        imgs_per_object: Imágenes por objeto (vistas diferentes)
        seed: Semilla global
    """
    print("=" * 60)
    print("  Generador de Dataset Sintético (Formato Cornell)")
    print("=" * 60)
    print(f"Objetos:           {n_objects}")
    print(f"Imgs por objeto:   {imgs_per_object}")
    print(f"Total imágenes:    {n_objects * imgs_per_object}")
    print(f"Directorio salida: {out_dir}")
    print(f"Semilla:           {seed}")
    print()
    
    out_dir.mkdir(parents=True, exist_ok=True)
    
    print("[1/2] Generando imágenes y anotaciones...")
    for obj_id in range(1, n_objects + 1):
        generate_synthetic_object(obj_id, imgs_per_object, out_dir, seed)
        if obj_id % 10 == 0:
            print(f"  Progreso: {obj_id}/{n_objects} objetos")
    
    print(f"[OK] {n_objects} objetos generados en {out_dir}")
    print()
    
    # Contar archivos
    n_rgb = len(list(out_dir.glob("**/*r.png")))
    n_depth = len(list(out_dir.glob("**/*d.png")))
    n_annot = len(list(out_dir.glob("**/*cpos.txt")))
    
    print(f"[OK] Archivos generados:")
    print(f"  RGB:         {n_rgb}")
    print(f"  Depth:       {n_depth}")
    print(f"  Anotaciones: {n_annot}")
    print()


def main():
    parser = argparse.ArgumentParser(
        description="Generar dataset sintético en formato Cornell"
    )
    parser.add_argument("--out-dir", type=str, required=True,
                        help="Directorio de salida (e.g., data/raw/cornell)")
    parser.add_argument("--n-objects", type=int, default=50,
                        help="Número de objetos (default: 50)")
    parser.add_argument("--imgs-per-object", type=int, default=5,
                        help="Imágenes por objeto (default: 5)")
    parser.add_argument("--seed", type=int, default=42,
                        help="Semilla para reproducibilidad")
    
    args = parser.parse_args()
    
    out_dir = Path(args.out_dir)
    
    generate_full_dataset(
        out_dir=out_dir,
        n_objects=args.n_objects,
        imgs_per_object=args.imgs_per_object,
        seed=args.seed
    )
    
    print("=" * 60)
    print("[OK] Dataset sintético generado")
    print("=" * 60)
    print()
    print("Siguiente paso: procesarlo con prepare_cornell_csv.py")
    print(f"  python3 scripts/prepare_cornell_csv.py \\")
    print(f"    --raw-dir {out_dir} \\")
    print(f"    --out-dir data/processed/cornell")
    print()


if __name__ == "__main__":
    main()
