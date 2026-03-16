#!/usr/bin/env python3
"""
Autonomous Touch Tuner - Calibration automática del probe de toque
Implementación desde cero optimizada para UR5 RG2
"""

import argparse
import csv
import json
import math
import os
import subprocess
import sys
import time
from dataclasses import dataclass, asdict
from datetime import datetime
from pathlib import Path
from typing import Optional, List, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from tf2_geometry_msgs import do_transform_pose_stamped
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


@dataclass
class TouchCalibrationPoint:
    """Punto de calibración del toque"""
    iteration: int
    target_x: float
    target_y: float
    target_z: float
    current_x: float
    current_y: float
    current_z: float
    error_xy: float
    error_z: float
    success: bool
    notes: str
    timestamp: float = 0.0


class AutoTuneTouchNode(Node):
    """Nodo principal para AUTO TUNE TOUCH"""

    def __init__(self,
                 base_frame: str = "base_link",
                 ee_frame: str = "rg2_tcp",
                 target_name: str = "FRONT_LEFT",
                 max_iterations: int = 50,
                 tol_xy: float = 0.01,
                 tol_z: float = 0.02):
        super().__init__("auto_tune_touch")
        
        self.base_frame = base_frame
        self.ee_frame = ee_frame
        self.target_name = target_name
        self.max_iterations = max_iterations
        self.tol_xy = tol_xy
        self.tol_z = tol_z
        
        # TF buffer
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Publishers para movimiento
        self.jt_pub = self.create_publisher(
            JointTrajectory,
            "/joint_trajectory_controller/joint_trajectory",
            10
        )
        self.gripper_pub = self.create_publisher(
            Float64MultiArray,
            "/gripper_controller/commands",
            10
        )
        
        # Historial de calibración
        self.calibration_history: List[TouchCalibrationPoint] = []
        self.results = {
            "success": False,
            "iterations": 0,
            "final_error_xy": float('inf'),
            "final_error_z": float('inf'),
            "points": []
        }

    def get_tcp_pose(self) -> Optional[Tuple[float, float, float]]:
        """Obtener posición actual del TCP en base_link"""
        try:
            now = self.get_clock().now()
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.ee_frame,
                now,
                timeout=Duration(seconds=1.0)
            )
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z
            return (x, y, z)
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return None

    def get_target_pose(self) -> Optional[Tuple[float, float, float]]:
        """Obtener pose objetivo basada en target_name"""
        # Puntos de calibración predefinidos (en base_link, metros)
        touch_points = {
            "FRONT_LEFT": (-0.15, 0.15, 0.05),
            "FRONT_CENTER": (0.0, 0.25, 0.05),
            "FRONT_RIGHT": (0.15, 0.15, 0.05),
            "LEFT": (-0.25, 0.0, 0.05),
            "CENTER": (0.0, 0.0, 0.05),
            "RIGHT": (0.25, 0.0, 0.05),
            "TABLE": (0.0, 0.0, -0.15),  # TOuch en surf
            "LOW": (0.0, 0.0, -0.20),     # Touch low
        }
        return touch_points.get(self.target_name, touch_points["CENTER"])

    def plan_movement_moveit(self, target_pose: Tuple[float, float, float]) -> bool:
        """Planificar movimiento con MoveIt para alcanzar pose objetivo"""
        try:
            # Llamar a MoveIt planificador (simplificado - requiere servicio)
            # Por ahora, usamos trajectoria directa (fallback)
            return self.send_direct_trajectory(target_pose)
        except Exception as e:
            self.get_logger().error(f"MoveIt planning failed: {e}")
            return False

    def send_direct_trajectory(self, target_pose: Tuple[float, float, float]) -> bool:
        """Enviar trayectoria directa (sin MoveIt) - solo para testing"""
        try:
            # Posición inicial "home" del robot
            home_joints = [0.0, -1.57, 1.57, -1.57, 1.57, 0.0]
            
            # Crear trayectoria con 2 puntos: home -> target
            traj = JointTrajectory()
            traj.header.frame_id = self.base_frame
            traj.header.stamp = self.get_clock().now().to_msg()
            traj.joint_names = [
                "shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint"
            ]
            
            # Punto inicial
            pt0 = JointTrajectoryPoint()
            pt0.positions = home_joints
            pt0.time_from_start.sec = 0
            pt0.time_from_start.nsec = 0
            
            # Punto objetivo (aproximado - requeriría IK completo)
            # Por ahora solo moveríamos Z
            pt1 = JointTrajectoryPoint()
            pt1.positions = home_joints.copy()
            pt1.positions[2] += 0.1  # Desciende 10cm aprox
            pt1.time_from_start.sec = 2
            
            traj.points = [pt0, pt1]
            
            # Publicar
            self.jt_pub.publish(traj)
            self.get_logger().info(f"Trajectory sent to {target_pose}")
            
            # Esperar a que ejecute
            rclpy.spin_once(self, timeout_sec=3.0)
            return True
            
        except Exception as e:
            self.get_logger().error(f"Direct trajectory failed: {e}")
            return False

    def run_calibration_loop(self) -> bool:
        """Ejecutar loop principal de calibración"""
        self.get_logger().info(
            f"Starting AUTO TUNE TOUCH: "
            f"target={self.target_name} "
            f"tol_xy={self.tol_xy:.4f} "
            f"tol_z={self.tol_z:.4f} "
            f"max_iter={self.max_iterations}"
        )
        
        target_pose = self.get_target_pose()
        if not target_pose:
            self.get_logger().error(f"Unknown target: {self.target_name}")
            return False
        
        target_x, target_y, target_z = target_pose
        self.get_logger().info(f"Target pose: ({target_x:.4f}, {target_y:.4f}, {target_z:.4f})")
        
        for iteration in range(self.max_iterations):
            self.get_logger().info(f"\n=== Iteration {iteration + 1}/{self.max_iterations} ===")
            
            # Obtener posición actual
            current_pose = self.get_tcp_pose()
            if not current_pose:
                self.get_logger().error("Cannot read TCP pose")
                break
            
            current_x, current_y, current_z = current_pose
            
            # Calcular erro
            error_xy = math.sqrt((current_x - target_x)**2 + (current_y - target_y)**2)
            error_z = abs(current_z - target_z)
            
            self.get_logger().info(
                f"Current: ({current_x:.4f}, {current_y:.4f}, {current_z:.4f}) "
                f"Error: XY={error_xy:.4f} Z={error_z:.4f}"
            )
            
            # Registrar punto
            point = TouchCalibrationPoint(
                iteration=iteration + 1,
                target_x=target_x,
                target_y=target_y,
                target_z=target_z,
                current_x=current_x,
                current_y=current_y,
                current_z=current_z,
                error_xy=error_xy,
                error_z=error_z,
                success=False,
                notes="",
                timestamp=time.time()
            )
            
            # Verificar convergencia
            if error_xy <= self.tol_xy and error_z <= self.tol_z:
                self.get_logger().info("✅ CALIBRATION CONVERGED!")
                point.success = True
                point.notes = "Converged"
                self.calibration_history.append(point)
                
                # Actualizar resultados
                self.results["success"] = True
                self.results["iterations"] = iteration + 1
                self.results["final_error_xy"] = error_xy
                self.results["final_error_z"] = error_z
                return True
            
            # Si no convergió, enviar movimiento correctivo
            # Calcular corrección (proporcional)
            k_xy = 0.5  # Ganancia XY
            k_z = 0.5   # Ganancia Z
            
            dx = (target_x - current_x) * k_xy
            dy = (target_y - current_y) * k_xy
            dz = (target_z - current_z) * k_z
            
            next_target = (
                current_x + dx,
                current_y + dy,
                current_z + dz
            )
            
            self.get_logger().info(f"Correction: dx={dx:.4f} dy={dy:.4f} dz={dz:.4f}")
            
            # Intentar movimiento
            if not self.plan_movement_moveit(next_target):
                point.notes = "Movement failed"
            else:
                point.notes = "Movement sent"
            
            self.calibration_history.append(point)
            
            # Pequeña pausa entre iteraciones
            time.sleep(1.0)
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # No convergió
        self.get_logger().warn("⚠️  Max iterations reached without convergence")
        if self.calibration_history:
            last = self.calibration_history[-1]
            self.results["final_error_xy"] = last.error_xy
            self.results["final_error_z"] = last.error_z
        self.results["iterations"] = len(self.calibration_history)
        
        return False

    def save_results(self, output_dir: Path) -> None:
        """Guardar resultados de calibración"""
        output_dir.mkdir(parents=True, exist_ok=True)
        
        # Guardar CSV
        csv_file = output_dir / "calibration.csv"
        with open(csv_file, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "iteration", "target_x", "target_y", "target_z",
                "current_x", "current_y", "current_z",
                "error_xy", "error_z", "success", "notes", "timestamp"
            ])
            for point in self.calibration_history:
                writer.writerow([
                    point.iteration, point.target_x, point.target_y, point.target_z,
                    point.current_x, point.current_y, point.current_z,
                    point.error_xy, point.error_z, point.success, point.notes, point.timestamp
                ])
        
        self.get_logger().info(f"CSV saved: {csv_file}")
        
        # Guardar JSON
        json_file = output_dir / "calibration_result.json"
        self.results["points"] = [asdict(p) for p in self.calibration_history]
        with open(json_file, "w") as f:
            json.dump(self.results, f, indent=2)
        
        self.get_logger().info(f"JSON saved: {json_file}")


def main():
    parser = argparse.ArgumentParser(description="AUTO TUNE TOUCH - Robot Calibration")
    parser.add_argument("--base_frame", default="base_link")
    parser.add_argument("--ee_frame", default="rg2_tcp")
    parser.add_argument("--target_name", default="FRONT_LEFT")
    parser.add_argument("--max_iterations", type=int, default=50)
    parser.add_argument("--max_iters", type=int, default=None)
    parser.add_argument("--tol_xy", type=float, default=0.01)
    parser.add_argument("--tol_z", type=float, default=0.02)
    parser.add_argument("--cartesian_descent", default="true")
    parser.add_argument("--constraint_orientation", default="true")
    parser.add_argument("--output_dir", default="/tmp/auto_tune_touch")
    
    args = parser.parse_args()
    
    rclpy.init(args=None)
    
    max_iterations = int(args.max_iters) if args.max_iters is not None else int(args.max_iterations)

    node = AutoTuneTouchNode(
        base_frame=args.base_frame,
        ee_frame=args.ee_frame,
        target_name=args.target_name,
        max_iterations=max_iterations,
        tol_xy=args.tol_xy,
        tol_z=args.tol_z
    )
    
    try:
        success = node.run_calibration_loop()
        node.save_results(Path(args.output_dir))
        
        print(f"\n{'='*60}")
        print(f"{'CALIBRATION RESULTS':^60}")
        print(f"{'='*60}")
        print(f"Status:         {'✅ SUCCESS' if success else '❌ FAILED'}")
        print(f"Iterations:     {node.results['iterations']}")
        print(f"Final Error XY: {node.results['final_error_xy']:.6f} m")
        print(f"Final Error Z:  {node.results['final_error_z']:.6f} m")
        print(f"Results saved:  {Path(args.output_dir).resolve()}")
        print(f"{'='*60}\n")
        
        return 0 if success else 1
        
    except KeyboardInterrupt:
        print("\n⏹️ Calibration interrupted by user")
        return 1
    except Exception as e:
        print(f"\n❌ Error: {e}")
        import traceback
        traceback.print_exc()
        return 1
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
