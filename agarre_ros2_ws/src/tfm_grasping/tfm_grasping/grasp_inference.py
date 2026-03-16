#!/usr/bin/env python3
"""
Nodo ROS2 para inferencia de agarres con modelo SimpleGraspCNN.
Suscribirse a imágenes RGB y publicar poses de agarre predichas.

Uso:
  ros2 run tfm_grasping grasp_inference
"""

import os
import sys
from pathlib import Path
from typing import Optional

try:
    import cv2
    import numpy as np
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from geometry_msgs.msg import Point
    from std_msgs.msg import Float32MultiArray
    from cv_bridge import CvBridge
    import torch
    from torchvision import transforms
except ImportError as e:
    print(f"❌ Dependencia faltante: {e}")
    sys.exit(1)

# Agregar src/ de agarre_inteligente al path
VISION_DIR = Path.home() / "TFM" / "agarre_inteligente"
if (VISION_DIR / "src").exists():
    sys.path.insert(0, str(VISION_DIR / "src"))

try:
    from graspnet.models.simple_grasp_cnn import SimpleGraspCNN
except ImportError as e:
    print(f"❌ No se pudo importar SimpleGraspCNN: {e}")
    print(f"   VISION_DIR: {VISION_DIR}")
    sys.exit(1)


class GraspInferenceNode(Node):
    """Nodo ROS2 para predicción de agarres en tiempo real."""

    def __init__(self):
        super().__init__("grasp_inference")
        
        # Parámetros
        self.img_size = 224
        self.declare_parameter("model_path", "models/exp1_simple_rgb_best.pth")
        self.declare_parameter("image_topic", "/camera/rgb/image_raw")
        self.declare_parameter("output_topic", "/tfm/grasp_prediction")
        self.declare_parameter("device", "auto")
        
        model_path = self.get_parameter("model_path").value
        image_topic = self.get_parameter("image_topic").value
        output_topic = self.get_parameter("output_topic").value
        device_str = self.get_parameter("device").value
        
        # Resolver device
        self.device = self._resolve_device(device_str)
        self.get_logger().info(f"🔧 Device: {self.device}")
        
        # Cargar modelo
        self.model = self._load_model(model_path)
        if self.model is None:
            self.get_logger().error("❌ No se pudo cargar el modelo. Abortando.")
            raise RuntimeError("Model loading failed")
        
        self.cv_bridge = CvBridge()
        
        # Transformaciones (compatibles con torchvision)
        self.transform = transforms.Compose([
            transforms.ToTensor(),
            transforms.Normalize(
                mean=[0.485, 0.456, 0.406],
                std=[0.229, 0.224, 0.225]
            ),
        ])
        
        # Suscriptores y publicadores
        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, 10
        )
        self.grasp_pub = self.create_publisher(
            Float32MultiArray, output_topic, 10
        )
        
        self.inference_count = 0
        self.get_logger().info(
            f"✅ GraspInferenceNode inicializado. "
            f"Suscrito a: {image_topic}, Publicando: {output_topic}"
        )

    def _resolve_device(self, device_str: str):
        """Determinar dispositivo (CPU/GPU)."""
        device_str = device_str.strip().lower()
        
        if device_str == "cpu":
            return torch.device("cpu")
        
        if device_str == "cuda":
            if not torch.cuda.is_available():
                self.get_logger().warn("CUDA solicitado pero no disponible. Usando CPU.")
                return torch.device("cpu")
            return torch.device("cuda")
        
        # Auto: usar GPU si disponible
        return torch.device("cuda" if torch.cuda.is_available() else "cpu")

    def _load_model(self, model_rel_path: str) -> Optional[torch.nn.Module]:
        """Cargar modelo entrenado."""
        try:
            # Construir ruta absoluta del modelo
            node_dir = Path(__file__).parent
            model_path = node_dir / model_rel_path
            
            if not model_path.exists():
                self.get_logger().error(f"Archivo no encontrado: {model_path}")
                return None
            
            # Crear modelo
            model = SimpleGraspCNN(in_channels=3)
            
            # Cargar pesos
            checkpoint = torch.load(str(model_path), map_location=self.device)
            
            # Si el checkpoint es un dict de state_dict, usarlo directo
            if isinstance(checkpoint, dict) and "state_dict" in checkpoint:
                model.load_state_dict(checkpoint["state_dict"])
            else:
                model.load_state_dict(checkpoint)
            
            # Preparar para inferencia
            model = model.to(self.device)
            model.eval()
            
            self.get_logger().info(f"✅ Modelo cargado correctamente: {model_path}")
            return model
            
        except Exception as e:
            self.get_logger().error(f"❌ Error cargando modelo: {e}")
            import traceback
            traceback.print_exc()
            return None

    def image_callback(self, msg: Image):
        """Procesar imagen RGB y publicar predicción."""
        if self.model is None:
            return
        
        try:
            # Convertir de ROS Image a OpenCV
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            
            # Redimensionar a 224x224
            cv_image_resized = cv2.resize(cv_image, (self.img_size, self.img_size))
            
            # Convertir BGR → RGB
            rgb_image = cv2.cvtColor(cv_image_resized, cv2.COLOR_BGR2RGB)
            
            # Normalizar y crear tensor
            tensor = torch.from_numpy(rgb_image).float() / 255.0  # [0, 1]
            tensor = tensor.permute(2, 0, 1)  # [H, W, C] → [C, H, W]
            tensor = tensor.unsqueeze(0).to(self.device)  # [1, C, H, W]
            
            # Inferencia
            with torch.no_grad():
                output = self.model(tensor)  # [1, 5]
            
            # Extraer parámetros
            params = output[0].cpu().numpy()  # [5]
            
            # Publicar resultado
            msg_out = Float32MultiArray()
            msg_out.data = [float(p) for p in params]
            self.grasp_pub.publish(msg_out)
            
            self.inference_count += 1
            if self.inference_count % 30 == 0:  # Log cada 30 frames
                cx, cy, w, h, angle = params
                self.get_logger().info(
                    f"📊 Inferencia #{self.inference_count}: "
                    f"cx={cx:.2f}, cy={cy:.2f}, w={w:.2f}, h={h:.2f}, "
                    f"angle={angle:.4f}"
                )
            
        except Exception as e:
            self.get_logger().error(f"❌ Error en image_callback: {e}")


def main(args=None):
    rclpy.init(args=args)
    try:
        node = GraspInferenceNode()
        rclpy.spin(node)
    except Exception as e:
        print(f"❌ Error fatal: {e}")
        import traceback
        traceback.print_exc()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
