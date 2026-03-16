#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/tools/extract_pick_comparison.py
# Contenido: Herramienta de comprobacion del workspace ROS 2.
# Uso breve: Se ejecuta manualmente para auditoria, checks fisicos o verificaciones funcionales.
"""
Script para extraer y analizar la comparativa de PICK MESA vs PICK Objeto
de los logs del panel.

Uso:
    python3 extract_pick_comparison.py [--log ARCHIVO] [--output ARCHIVO]

Extrae información sobre:
    - Secuencia de joints (J1-J6)
    - Poses cartesianas enviadas a MoveIt
    - Orientaciones de gripper
    - TCP tracking
    - Comparativa PICK MESA vs PICK Objeto
"""

import argparse
import re
from pathlib import Path
from datetime import datetime
from typing import List, Tuple, Optional, Dict


class PickAnalyzer:
    """Analizador de logs de PICK MESA y PICK Objeto."""
    
    def __init__(self, log_file: str):
        self.log_file = Path(log_file)
        self.lines = []
        self.pick_mesa_events = []
        self.pick_objeto_events = []
        self.tcp_events = []
        
        self._load_log()
    
    def _load_log(self):
        """Carga el archivo de log."""
        if not self.log_file.exists():
            raise FileNotFoundError(f"Archivo de log no encontrado: {self.log_file}")
        
        with open(self.log_file, 'r') as f:
            self.lines = f.readlines()
        
        print(f"✓ Log cargado: {len(self.lines)} líneas")
    
    def extract_pick_mesa(self) -> List[str]:
        """Extrae eventos de PICK MESA."""
        events = []
        in_pick_mesa = False
        
        for i, line in enumerate(self.lines):
            if '[PICK] Paso joint: HOME' in line and '[DEMO]' in line:
                in_pick_mesa = True
                start_line = i
            
            if in_pick_mesa:
                events.append(line.strip())
                
                if '[PICK] Secuencia PICK completada' in line:
                    in_pick_mesa = False
                    print(f"✓ PICK MESA encontrado: líneas {start_line} a {i}")
                    break
        
        self.pick_mesa_events = events
        return events
    
    def extract_pick_objeto(self) -> List[str]:
        """Extrae eventos de PICK Objeto."""
        events = []
        in_pick_objeto = False
        
        for i, line in enumerate(self.lines):
            if '[PICK] Click en objeto:' in line:
                in_pick_objeto = True
                start_line = i
            
            if in_pick_objeto:
                events.append(line.strip())
                
                # PICK Objeto puede estar bloqueado o ejecutarse
                if '[PICK_OBJ]' in line or 'Bloqueado' in line:
                    continue
                
                if len(events) > 20:  # Límite para evitar extracción excesiva
                    break
        
        self.pick_objeto_events = events
        return events
    
    def extract_tcp_tracking(self) -> List[Tuple[str, float, float, float]]:
        """Extrae posiciones TCP durante la ejecución."""
        tcp_data = []
        
        pattern = r'\[TRACE\].*TCP/world:\s*\(([-\d.]+),\s*([-\d.]+),\s*([-\d.]+)\)'
        
        for line in self.lines:
            match = re.search(pattern, line)
            if match:
                x, y, z = float(match.group(1)), float(match.group(2)), float(match.group(3))
                tcp_data.append((line.strip(), x, y, z))
        
        self.tcp_events = tcp_data
        return tcp_data
    
    def extract_joint_targets(self) -> Dict[str, List[float]]:
        """Extrae los targets de joints."""
        joints = {}
        
        # Definiciones de joints del código
        joints_def = {
            'HOME': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            'MESA': [-2.7, -0.1, 90.0, -5.5, -90.7, 0.9],
            'PICK_IMAGE': [-15.7, -1.3, 110.9, -17.5, -89.3, 0.9],
            'CESTA': [167.6, -4.8, 110.9, -21.1, -92.0, 0.9],
            'BASKET': [167.6, -4.8, 110.9, -21.1, -92.0, 0.9],
        }
        
        return joints_def
    
    def generate_report(self, output_file: Optional[str] = None) -> str:
        """Genera un reporte detallado."""
        report = []
        report.append("=" * 80)
        report.append("ANÁLISIS COMPARATIVO: PICK MESA vs PICK OBJETO")
        report.append("=" * 80)
        report.append(f"\nArch ivo de log: {self.log_file}")
        report.append(f"Fecha de análisis: {datetime.now().isoformat()}")
        report.append(f"\nTotal de líneas: {len(self.lines)}")
        
        # PICK MESA
        report.append("\n" + "=" * 80)
        report.append("PICK MESA - EJECUCIÓN")
        report.append("=" * 80)
        
        if self.pick_mesa_events:
            report.append(f"\nEventos encontrados: {len(self.pick_mesa_events)}")
            report.append("\nPrimeros 20 eventos:")
            for i, event in enumerate(self.pick_mesa_events[:20]):
                if '[PICK]' in event or '[DEMO]' in event or '[GRIPPER]' in event:
                    report.append(f"  {event}")
        else:
            report.append("\n⚠ No se encontraron eventos de PICK MESA")
        
        # PICK Objeto
        report.append("\n" + "=" * 80)
        report.append("PICK OBJETO - ESTADO")
        report.append("=" * 80)
        
        if self.pick_objeto_events:
            report.append(f"\nEventos encontrados: {len(self.pick_objeto_events)}")
            report.append("\nEventos:")
            for event in self.pick_objeto_events[:15]:
                report.append(f"  {event}")
        else:
            report.append("\n⚠ No se encontraron eventos de PICK Objeto")
        
        # TCP Tracking
        report.append("\n" + "=" * 80)
        report.append("TCP TRACKING DURANTE SIMULACIÓN")
        report.append("=" * 80)
        
        if self.tcp_events:
            report.append(f"\nRegistros TCP encontrados: {len(self.tcp_events)}")
            report.append("\nPrimeros 10 registros:")
            for x, y, z in [(tcp[1], tcp[2], tcp[3]) for tcp in self.tcp_events[:10]]:
                report.append(f"  TCP/world: ({x:7.3f}, {y:7.3f}, {z:7.3f})")
        else:
            report.append("\n⚠ No se encontraron registros de TCP")
        
        # Joint Targets
        report.append("\n" + "=" * 80)
        report.append("VALORES DE JOINTS OBJETIVO (J1-J6)")
        report.append("=" * 80)
        
        joints = self.extract_joint_targets()
        report.append("\n" + "Pose".ljust(15) + "J1 (pan)".rjust(12) + "J2 (lift)".rjust(12) + 
                     "J3 (elbow)".rjust(12) + "J4 (w1)".rjust(12) + "J5 (w2)".rjust(12) + "J6 (w3)".rjust(12))
        report.append("-" * 80)
        
        for pose_name, joint_values in joints.items():
            j1, j2, j3, j4, j5, j6 = joint_values
            report.append(f"{pose_name.ljust(15)}{j1:12.1f}{j2:12.1f}{j3:12.1f}{j4:12.1f}{j5:12.1f}{j6:12.1f}")
        
        # Conclusiones
        report.append("\n" + "=" * 80)
        report.append("CONCLUSIONES")
        report.append("=" * 80)
        
        has_pick_mesa = len(self.pick_mesa_events) > 0
        has_pick_objeto = len(self.pick_objeto_events) > 0
        
        report.append(f"\n✓ PICK MESA: {'EJECUTADO' if has_pick_mesa else 'NO ENCONTRADO'}")
        report.append(f"✓ PICK Objeto: {'ENCONTRADO' if has_pick_objeto else 'NO ENCONTRADO'}")
        report.append(f"✓ TCP tracking: {'DISPONIBLE' if len(self.tcp_events) > 0 else 'NO DISPONIBLE'}")
        
        report.append("\nNotas:")
        report.append("  - J5 (wrist_2) debe estar ≈ -90° para gripper vertical")
        report.append("  - Valores de J1, J3, J4 varían según posición del objeto")
        report.append("  - TCP debe descender sobre objeto en GRASP_DOWN")
        
        report_text = "\n".join(report)
        
        if output_file:
            with open(output_file, 'w') as f:
                f.write(report_text)
            print(f"\n✓ Reporte guardado en: {output_file}")
        
        return report_text


def main():
    parser = argparse.ArgumentParser(
        description='Extrae y analiza comparativa de PICK MESA vs PICK Objeto'
    )
    parser.add_argument(
        '--log',
        type=str,
        default='/home/laboratorio/TFM/auditoria/panel_run.log',
        help='Archivo de log del panel (default: auditoria/panel_run.log)'
    )
    parser.add_argument(
        '--output',
        type=str,
        help='Archivo donde guardar el reporte (opcional)'
    )
    
    args = parser.parse_args()
    
    try:
        analyzer = PickAnalyzer(args.log)
        
        print("\n[EXTRAYENDO DATOS...]")
        analyzer.extract_pick_mesa()
        analyzer.extract_pick_objeto()
        analyzer.extract_tcp_tracking()
        
        print("\n[GENERANDO REPORTE...]")
        report = analyzer.generate_report(args.output)
        
        print("\n" + report)
        
    except FileNotFoundError as e:
        print(f"ERROR: {e}")
        return 1
    except Exception as e:
        print(f"ERROR inesperado: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    return 0


if __name__ == '__main__':
    exit(main())
