"""F11 (auditoría 2026-05-10): namespace de utilities base.

Subpaquete que re-exporta utilities transversales usadas por varios
dominios:
  * ``param_utils`` — read_*_param helpers.
  * ``tf_batch_lookups`` / ``tf_batch_lookups_runtime`` — batch TF.

Los archivos físicos siguen en top-level; F11 iter 2 los moverá.
"""
from __future__ import annotations

from ..param_utils import (  # noqa: F401
    read_float_param,
    read_int_param,
    read_str_list_param,
    read_str_param,
)

__all__ = [
    "read_float_param",
    "read_int_param",
    "read_str_list_param",
    "read_str_param",
]
