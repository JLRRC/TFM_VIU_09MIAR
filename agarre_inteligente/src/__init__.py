"""
Agarre Inteligente
==================

Sistema de Deep Learning para clasificación de agarres robóticos.

Este paquete contiene todos los módulos necesarios para entrenar,
evaluar y utilizar modelos de visión por computador para predecir
la viabilidad de agarres robóticos a partir de imágenes RGB y RGB-D.

Módulos:
--------
- models: Definiciones de arquitecturas de redes neuronales
- data: Datasets y dataloaders personalizados
- training: Lógica de entrenamiento y optimización
- evaluation: Evaluación y métricas de modelos
- utils: Utilidades (config, logging, checkpoints)

Autor: Laboratorio de Robótica
Fecha: Marzo 2026
"""

__version__ = "1.0.0"
__author__ = "Laboratorio de Robótica"
__email__ = "laboratorio@example.com"

__all__ = [
    "models",
    "data",
    "training",
    "evaluation",
    "utils",
]
