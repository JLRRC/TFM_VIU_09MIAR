#!/usr/bin/env python3
# Ruta/archivo: agarre_ros2_ws/tools/monitor_joints.py
# Contenido: Herramienta de comprobacion del workspace ROS 2.
# Uso breve: Se ejecuta manualmente para auditoria, checks fisicos o verificaciones funcionales.
"""
Script para monitorizar en tiempo real los valores de los joints J1-J6 del UR5.
Se suscribe a /joint_states y muestra los valores en grados.

Uso:
    python3 monitor_joints.py [--log ARCHIVO]
"""
import argparse
import sys
import math
from datetime import datetime

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
except ImportError as e:
    print(f"ERROR: No se puede importar ROS 2: {e}", file=sys.stderr)
    print("Asegúrate de haber ejecutado: source /opt/ros/jazzy/setup.bash", file=sys.stderr)
    sys.exit(1)


JOINT_NAMES = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]


class JointMonitor(Node):
    def __init__(self, log_file=None):
        super().__init__('joint_monitor')
        self.log_file = log_file
        self.log_fp = None
        
        if self.log_file:
            try:
                self.log_fp = open(self.log_file, 'a')
                self.log_fp.write(f"\n{'='*80}\n")
                self.log_fp.write(f"Inicio monitoreo: {datetime.now().isoformat()}\n")
                self.log_fp.write(f"{'='*80}\n\n")
                self.log_fp.flush()
            except Exception as e:
                self.get_logger().error(f"No se pudo abrir archivo de log: {e}")
                self.log_fp = None
        
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )
        self.last_positions = None
        self.count = 0
        
        self.get_logger().info('Monitor de joints iniciado. Esperando mensajes en /joint_states...')
        if self.log_file:
            self.get_logger().info(f'Guardando log en: {self.log_file}')

    def joint_callback(self, msg):
        """Callback que se ejecuta cada vez que llega un mensaje de joint_states."""
        self.count += 1
        
        # Extraer posiciones de los 6 joints del UR5
        positions = {}
        for joint_name in JOINT_NAMES:
            if joint_name in msg.name:
                idx = msg.name.index(joint_name)
                if idx < len(msg.position):
                    positions[joint_name] = msg.position[idx]
        
        if len(positions) != 6:
            # No todos los joints presentes, skip
            return
        
        # Convertir a grados
        positions_deg = {name: math.degrees(rad) for name, rad in positions.items()}
        
        # Formatear salida
        timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
        
        # Línea compacta
        j1 = positions_deg['shoulder_pan_joint']
        j2 = positions_deg['shoulder_lift_joint']
        j3 = positions_deg['elbow_joint']
        j4 = positions_deg['wrist_1_joint']
        j5 = positions_deg['wrist_2_joint']
        j6 = positions_deg['wrist_3_joint']
        
        output = (
            f"[{timestamp}] "
            f"J1={j1:7.2f}° J2={j2:7.2f}° J3={j3:7.2f}° "
            f"J4={j4:7.2f}° J5={j5:7.2f}° J6={j6:7.2f}°"
        )
        
        # Mostrar solo si hay cambio significativo (> 0.5 grados)
        if self.last_positions is not None:
            max_diff = 0.0
            for name in JOINT_NAMES:
                diff = abs(positions_deg[name] - self.last_positions[name])
                max_diff = max(max_diff, diff)
            
            if max_diff < 0.5:
                return  # Sin cambio significativo, no mostrar
        
        print(output)
        sys.stdout.flush()
        
        if self.log_fp:
            self.log_fp.write(output + '\n')
            self.log_fp.flush()
        
        self.last_positions = positions_deg

    def __del__(self):
        if self.log_fp:
            self.log_fp.write(f"\n{'='*80}\n")
            self.log_fp.write(f"Fin monitoreo: {datetime.now().isoformat()}\n")
            self.log_fp.write(f"{'='*80}\n")
            self.log_fp.close()


def main():
    parser = argparse.ArgumentParser(
        description='Monitorea los valores de joints J1-J6 del UR5 en tiempo real'
    )
    parser.add_argument(
        '--log',
        type=str,
        help='Archivo donde guardar el log de joints (opcional)'
    )
    args = parser.parse_args()
    
    rclpy.init()
    monitor = JointMonitor(log_file=args.log)
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        print('\n\nMonitoreo detenido por usuario.')
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
