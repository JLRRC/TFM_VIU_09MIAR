"""F5-step4: tests puros del cache de poses por nombre.

Cubre ObjectPoseCache + transform_to_pose7 + tfmessage_to_updates.
Stdlib + types.SimpleNamespace para emular ROS msgs sin importarlos.
"""

from __future__ import annotations

from types import SimpleNamespace

import pytest

from ur5_tools.object_pose_cache import (
    CachedPose,
    ObjectPoseCache,
    tfmessage_to_updates,
    transform_to_pose7,
)


# ---------------------------------------------------------------------------
# CachedPose
# ---------------------------------------------------------------------------


def test_cached_pose_dataclass_attributes():
    cp = CachedPose(
        pose7=(0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0),
        stamp_sec=100.0,
        frame_id="world",
    )
    assert cp.pose7 == (0.1, 0.2, 0.3, 0.0, 0.0, 0.0, 1.0)
    assert cp.stamp_sec == 100.0
    assert cp.frame_id == "world"


def test_cached_pose_default_frame_id():
    cp = CachedPose(pose7=(0.0,) * 7, stamp_sec=0.0)
    assert cp.frame_id == "world"


def test_cached_pose_is_frozen():
    cp = CachedPose(pose7=(0.0,) * 7, stamp_sec=0.0)
    with pytest.raises(Exception):
        cp.stamp_sec = 999.0


# ---------------------------------------------------------------------------
# ObjectPoseCache.update + lookup
# ---------------------------------------------------------------------------


def test_lookup_unknown_returns_reason():
    c = ObjectPoseCache()
    cached, reason = c.lookup("missing", now_sec=1.0, max_age_sec=1.0)
    assert cached is None
    assert reason == "object_not_in_cache"


def test_update_then_lookup_returns_cached_pose():
    c = ObjectPoseCache()
    pose = (0.5, -0.1, 0.05, 0.0, 0.0, 0.0, 1.0)
    c.update("box_red", pose, stamp_sec=10.0, frame_id="world")
    cached, reason = c.lookup("box_red", now_sec=10.5, max_age_sec=1.0)
    assert reason == ""
    assert cached is not None
    assert cached.pose7 == pose
    assert cached.stamp_sec == 10.0
    assert cached.frame_id == "world"


def test_lookup_stale_returns_none_and_reason():
    c = ObjectPoseCache()
    c.update("box", (0.0,) * 7, stamp_sec=10.0)
    cached, reason = c.lookup("box", now_sec=12.0, max_age_sec=1.0)
    assert cached is None
    assert "pose_stale" in reason
    assert "max=1.000" in reason


def test_lookup_max_age_zero_disables_freshness_check():
    c = ObjectPoseCache()
    c.update("box", (0.0,) * 7, stamp_sec=10.0)
    cached, reason = c.lookup("box", now_sec=999.0, max_age_sec=0.0)
    assert reason == ""
    assert cached is not None


def test_update_overwrites_previous_pose():
    c = ObjectPoseCache()
    c.update("box", (0.0,) * 7, stamp_sec=1.0)
    c.update("box", (1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0), stamp_sec=2.0)
    cached, _ = c.lookup("box", now_sec=2.0, max_age_sec=10.0)
    assert cached.pose7 == (1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0)
    assert cached.stamp_sec == 2.0


def test_update_rejects_empty_name():
    c = ObjectPoseCache()
    c.update("", (0.0,) * 7, stamp_sec=1.0)
    assert c.known_objects() == []


def test_update_rejects_invalid_pose_length():
    c = ObjectPoseCache()
    c.update("box", (0.0, 1.0, 2.0), stamp_sec=1.0)  # len 3
    assert c.known_objects() == []


def test_update_rejects_non_numeric_pose():
    c = ObjectPoseCache()
    c.update("box", ("a", "b", "c", "d", "e", "f", "g"), stamp_sec=1.0)
    assert c.known_objects() == []


def test_known_objects_sorted():
    c = ObjectPoseCache()
    c.update("zeta", (0.0,) * 7, stamp_sec=1.0)
    c.update("alpha", (0.0,) * 7, stamp_sec=1.0)
    c.update("mu", (0.0,) * 7, stamp_sec=1.0)
    assert c.known_objects() == ["alpha", "mu", "zeta"]


def test_clear_removes_everything():
    c = ObjectPoseCache()
    c.update("box1", (0.0,) * 7, stamp_sec=1.0)
    c.update("box2", (0.0,) * 7, stamp_sec=1.0)
    c.clear()
    assert c.known_objects() == []


# ---------------------------------------------------------------------------
# transform_to_pose7
# ---------------------------------------------------------------------------


def _mock_transform(tx, ty, tz, qx, qy, qz, qw):
    return SimpleNamespace(
        translation=SimpleNamespace(x=tx, y=ty, z=tz),
        rotation=SimpleNamespace(x=qx, y=qy, z=qz, w=qw),
    )


def test_transform_to_pose7_returns_seven_floats():
    t = _mock_transform(0.1, 0.2, 0.3, 0.0, 0.0, 0.7071, 0.7071)
    pose = transform_to_pose7(t)
    assert pose == (0.1, 0.2, 0.3, 0.0, 0.0, 0.7071, 0.7071)


def test_transform_to_pose7_none_returns_none():
    assert transform_to_pose7(None) is None


def test_transform_to_pose7_missing_translation_returns_none():
    t = SimpleNamespace(rotation=SimpleNamespace(x=0, y=0, z=0, w=1))
    assert transform_to_pose7(t) is None


def test_transform_to_pose7_missing_rotation_returns_none():
    t = SimpleNamespace(translation=SimpleNamespace(x=0, y=0, z=0))
    assert transform_to_pose7(t) is None


# ---------------------------------------------------------------------------
# tfmessage_to_updates
# ---------------------------------------------------------------------------


def _mock_tfmessage(items):
    """items: list of (name, pos_tuple3, quat_tuple4, frame_id)."""
    transforms = []
    for name, pos, quat, frame_id in items:
        transforms.append(SimpleNamespace(
            child_frame_id=name,
            header=SimpleNamespace(frame_id=frame_id),
            transform=SimpleNamespace(
                translation=SimpleNamespace(x=pos[0], y=pos[1], z=pos[2]),
                rotation=SimpleNamespace(x=quat[0], y=quat[1], z=quat[2], w=quat[3]),
            ),
        ))
    return SimpleNamespace(transforms=transforms)


def test_tfmessage_to_updates_yields_one_per_transform():
    msg = _mock_tfmessage([
        ("box_red", (0.5, 0.0, 0.05), (0.0, 0.0, 0.0, 1.0), "world"),
        ("box_blue", (0.4, 0.1, 0.05), (0.0, 0.0, 0.0, 1.0), "world"),
    ])
    out = tfmessage_to_updates(msg, now_sec=99.0)
    assert len(out) == 2
    assert out[0][0] == "box_red"
    assert out[0][1] == (0.5, 0.0, 0.05, 0.0, 0.0, 0.0, 1.0)
    assert out[0][2] == 99.0
    assert out[0][3] == "world"


def test_tfmessage_to_updates_skips_empty_child_frame_id():
    msg = _mock_tfmessage([
        ("", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), "world"),
        ("ok", (1.0, 2.0, 3.0), (0.0, 0.0, 0.0, 1.0), "world"),
    ])
    out = tfmessage_to_updates(msg, now_sec=1.0)
    assert len(out) == 1
    assert out[0][0] == "ok"


def test_tfmessage_to_updates_handles_none():
    assert tfmessage_to_updates(None, now_sec=0.0) == []


def test_tfmessage_to_updates_handles_no_transforms_attr():
    out = tfmessage_to_updates(SimpleNamespace(), now_sec=0.0)
    assert out == []


def test_tfmessage_to_updates_uses_callback_now_sec_not_header():
    """El stamp_sec siempre es now_sec, no del header del msg."""
    msg = _mock_tfmessage([
        ("box", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), "world"),
    ])
    out = tfmessage_to_updates(msg, now_sec=12345.0)
    assert out[0][2] == 12345.0


def test_tfmessage_to_updates_default_frame_id_when_missing():
    transforms = [SimpleNamespace(
        child_frame_id="box",
        # header sin frame_id
        header=SimpleNamespace(),
        transform=SimpleNamespace(
            translation=SimpleNamespace(x=0.0, y=0.0, z=0.0),
            rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        ),
    )]
    msg = SimpleNamespace(transforms=transforms)
    out = tfmessage_to_updates(msg, now_sec=1.0)
    assert out[0][3] == "world"
