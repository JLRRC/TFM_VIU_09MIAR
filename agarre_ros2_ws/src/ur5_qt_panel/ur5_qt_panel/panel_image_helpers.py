#!/usr/bin/env python3
"""F9 (auditoría 2026-05-10): helpers puros de procesamiento de imagen.

Funciones de cálculo numérico/cv2 extraídas del god-class
``RosWorker`` (panel_ros.py) que originalmente vivían enterradas en
métodos de instancia mezclando matemática, cache, locks y signal
emission. Cada helper es 100% puro: dado un input, devuelve un output
sin tocar estado mutable, ROS, Qt ni filesystem.

Funciones:
  * ``compute_depth_normalization_range(arr, *, fast=False, stride=1)``
    → ``(lo, hi)`` para normalizar un depth array float, usando
    percentiles 1%/99% (robusto) o min/max (rápido).
  * ``normalize_depth_uint8(arr, lo, hi)`` → uint8 array con clamp [0,255].
  * ``colorize_depth_bgr(arr, lo, hi)`` → BGR array con colormap TURBO.
  * ``resize_image_max_dim(bgr, max_size)`` → resize manteniendo aspect
    si ``max(h, w) > max_size``; devuelve el array original si no.

Imports lazy (``numpy``, ``cv2``) dentro de cada función para que el
módulo sea importable incluso si numpy/cv2 no están disponibles —
útil para tests offline y para entornos parciales (los wrappers en
RosWorker mantienen el comportamiento "no-op si bridge no inicializa").
"""
from __future__ import annotations

from typing import Any, Optional, Tuple


def compute_depth_normalization_range(
    arr: Any,
    *,
    fast: bool = False,
    stride: int = 1,
) -> Optional[Tuple[float, float]]:
    """Calcula ``(lo, hi)`` de un depth array float para normalización.

    Args:
        arr: ``numpy.ndarray`` 2D float (depth en metros u otra unidad).
            Pixels <= 0 se descartan (suelen ser "no medición").
        fast: si True, usa min/max directos (más rápido pero sensible
            a outliers). Si False, usa percentiles 1% y 99%.
        stride: subsampleo del array (``arr[::stride, ::stride]``) para
            acelerar el cálculo cuando la imagen es grande. ``stride>=1``.

    Returns:
        ``(lo, hi)`` con los floats del rango. ``None`` si el array no
        contiene pixels válidos (todos <= 0 o array vacío).

    Notes:
        El caller es responsable de cachear el rango si quiere
        amortizar el coste entre frames consecutivos. Esta función NO
        cachea — recalcula cada llamada.
    """
    import numpy as np

    if arr is None:
        return None
    s = max(1, int(stride))
    sample = arr[::s, ::s]
    sample = sample[sample > 0.0]
    if not sample.size:
        return None
    if fast:
        return float(sample.min()), float(sample.max())
    return float(np.percentile(sample, 1.0)), float(np.percentile(sample, 99.0))


def normalize_depth_uint8(arr: Any, lo: float, hi: float) -> Any:
    """Normaliza un depth array float a uint8 con clamp [0, 255].

    Args:
        arr: ``numpy.ndarray`` float 2D.
        lo, hi: rango de normalización. Si ``hi <= lo`` se ajusta
            ``hi = lo + 1e-3`` para evitar división por cero.

    Returns:
        ``numpy.ndarray`` uint8 del mismo shape que ``arr``.
    """
    import numpy as np

    if hi <= lo:
        hi = lo + 1e-3
    norm = (arr - lo) / (hi - lo)
    norm = np.clip(norm, 0.0, 1.0)
    return (norm * 255.0).astype("uint8")


def colorize_depth_bgr(arr: Any, lo: float, hi: float) -> Any:
    """Aplica TURBO colormap a un depth array tras normalizar a uint8.

    Conveniencia para visualización en cámara: pipeline completo
    ``arr float → uint8 normalizado → BGR colorizado``.
    """
    import cv2

    u8 = normalize_depth_uint8(arr, lo, hi)
    return cv2.applyColorMap(u8, cv2.COLORMAP_TURBO)


def resize_image_max_dim(bgr: Any, max_size: int) -> Any:
    """Resize ``bgr`` manteniendo aspect ratio si ``max(h, w) > max_size``.

    Args:
        bgr: ``numpy.ndarray`` HxWxC (típicamente 3 canales BGR).
        max_size: dimensión máxima en píxeles para ``max(h, w)``. Si
            ``<= 0`` o si la imagen ya es menor, devuelve ``bgr`` sin
            tocar (no copia).

    Returns:
        ``numpy.ndarray`` redimensionado (cv2.INTER_AREA) o el original.
    """
    import cv2

    if bgr is None or max_size is None or int(max_size) <= 0:
        return bgr
    h, w = bgr.shape[:2]
    cap = int(max_size)
    if max(h, w) <= cap:
        return bgr
    scale = cap / float(max(h, w))
    new_w = max(1, int(w * scale))
    new_h = max(1, int(h * scale))
    return cv2.resize(bgr, (new_w, new_h), interpolation=cv2.INTER_AREA)


__all__ = [
    "compute_depth_normalization_range",
    "normalize_depth_uint8",
    "colorize_depth_bgr",
    "resize_image_max_dim",
]
