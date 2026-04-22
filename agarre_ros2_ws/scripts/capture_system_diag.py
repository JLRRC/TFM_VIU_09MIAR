#!/usr/bin/env python3
"""Capture one /system_diag payload and optionally enforce readiness conditions."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time
from typing import Any, Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


class SystemDiagCapture(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("capture_system_diag")
        self.payload: Optional[dict[str, Any]] = None
        self.error: str = ""
        self.create_subscription(String, topic, self._on_msg, 10)

    def _on_msg(self, msg: String) -> None:
        raw = str(getattr(msg, "data", "") or "").strip()
        if not raw:
            self.error = "empty_payload"
            return
        try:
            data = json.loads(raw)
        except Exception as exc:
            self.error = f"invalid_json:{exc}"
            return
        if not isinstance(data, dict):
            self.error = "payload_not_object"
            return
        self.payload = data


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Capture a /system_diag JSON payload")
    parser.add_argument("--topic", default="/system_diag", help="Topic to subscribe to")
    parser.add_argument("--timeout", type=float, default=10.0, help="Timeout in seconds")
    parser.add_argument("--output", help="Optional JSON output file")
    parser.add_argument(
        "--require-geometry-ok",
        action="store_true",
        help="Fail if geometry_ok is not true",
    )
    parser.add_argument(
        "--require-state",
        action="append",
        default=[],
        help="Allowed system_state value. May be passed multiple times.",
    )
    parser.add_argument(
        "--quiet",
        action="store_true",
        help="Do not print the captured JSON payload",
    )
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    rclpy.init()
    node = SystemDiagCapture(args.topic)
    payload: Optional[dict[str, Any]] = None
    deadline = time.monotonic() + max(0.1, float(args.timeout))
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
            if node.payload is not None:
                payload = node.payload
                break
            if node.error:
                break
    except ExternalShutdownException:
        node.error = "external_shutdown"
    finally:
        node.destroy_node()
        try:
            rclpy.try_shutdown()
        except Exception:
            pass

    if payload is None:
        if node.error:
            print(f"[SYSTEM_DIAG][ERROR] {node.error}")
        else:
            print(f"[SYSTEM_DIAG][ERROR] timeout topic={args.topic} timeout={args.timeout:.1f}s")
        return 1

    if args.require_geometry_ok and not bool(payload.get("geometry_ok")):
        print(
            "[SYSTEM_DIAG][ERROR] geometry_ok=false "
            f"reason={payload.get('geometry_reason') or payload.get('reason') or 'unknown'}"
        )
        if args.output:
            Path(args.output).write_text(json.dumps(payload, indent=2), encoding="utf-8")
        return 1

    if args.require_state:
        allowed_states = {str(state).strip() for state in args.require_state if str(state).strip()}
        current_state = str(payload.get("state") or "").strip()
        if current_state not in allowed_states:
            print(
                "[SYSTEM_DIAG][ERROR] unexpected_state "
                f"current={current_state or 'none'} "
                f"allowed={','.join(sorted(allowed_states))}"
            )
            if args.output:
                Path(args.output).write_text(json.dumps(payload, indent=2), encoding="utf-8")
            return 1

    if args.output:
        out_path = Path(args.output).expanduser().resolve()
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    if not args.quiet:
        print(json.dumps(payload, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
