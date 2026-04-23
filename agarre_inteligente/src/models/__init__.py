"""
Módulo de modelos de redes neuronales.

Contiene las definiciones de arquitecturas:
- SimpleCNN: baseline ligera existente
- SimpleGrasp: variante alineada con la arquitectura descrita en el TFM
- ResNet variants: ResNet18 adaptado para agarres
"""

from .factory import build_model
from .simple_cnn import SimpleCNN
from .simple_grasp import SimpleGrasp
from .resnet_variants import ResNetGrasp

__all__ = ["build_model", "SimpleCNN", "SimpleGrasp", "ResNetGrasp"]
