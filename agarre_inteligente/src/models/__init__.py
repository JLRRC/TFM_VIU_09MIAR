"""
Módulo de modelos de redes neuronales.

Contiene las definiciones de arquitecturas:
- SimpleCNN: CNN simple para clasificación binaria
- ResNet variants: ResNet18 adaptado para agarres
"""

from .simple_cnn import SimpleCNN
from .resnet_variants import ResNetGrasp

__all__ = ["SimpleCNN", "ResNetGrasp"]
