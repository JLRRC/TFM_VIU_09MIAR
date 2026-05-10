"""F11 (auditoría 2026-05-10): namespace de gripper + attach del stack UR5.

Subpaquete que re-exporta los módulos del dominio "gripper attach +
release de objetos":
  * ``gripper_attach_backend`` — LifecycleNode con services attach/detach.
  * ``gripper_attach_models`` — dataclasses puras del backend.
  * ``attach_*`` — helpers internos del attach backend.
  * ``release_objects_service`` — LifecycleNode de release.
  * ``release_objects_logic`` / ``release_objects_geometry`` — helpers puros.

Los archivos físicos siguen en top-level; F11 iter 2 los moverá.
"""
from __future__ import annotations

from ..release_objects_geometry import (  # noqa: F401
    is_pose_on_table,
    parse_table_geometry_from_sdf,
    pose_tuple_from_text,
    quat_from_rpy,
)
from ..release_objects_logic import (  # noqa: F401
    compute_missing_required,
    drop_anchor_cleanup_targets,
    find_drop_anchor_duplicates,
    parse_world_name_from_sdf,
    pick_gz_service,
)

__all__ = [
    # geometry
    "is_pose_on_table",
    "parse_table_geometry_from_sdf",
    "pose_tuple_from_text",
    "quat_from_rpy",
    # logic
    "compute_missing_required",
    "drop_anchor_cleanup_targets",
    "find_drop_anchor_duplicates",
    "parse_world_name_from_sdf",
    "pick_gz_service",
]
