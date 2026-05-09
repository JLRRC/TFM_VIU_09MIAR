#!/usr/bin/env python3
"""
capture_camera_frames.py — Captura frames de cámara durante el ciclo de pick.

Suscribe a /camera_overhead/image y /camera_north/image.
Guarda JPEG cada --interval segundos mientras se ejecuta.
Guarda frames extra cuando detecta cambio en /gripper_cmd o /attach_state.

Uso:
    python3 capture_camera_frames.py --output DIR [--interval 2.0] [--timeout 600]
"""

import argparse
import os
import sys
import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


def _ros_image_to_jpeg_bytes(msg: Image) -> bytes | None:
    """Convierte sensor_msgs/Image RGB8 a JPEG bytes. Devuelve None si falla."""
    if msg.width == 0 or msg.height == 0:
        return None
    try:
        import struct
        # Try PIL first, then fallback to PPM
        try:
            from PIL import Image as PILImage
            import io
            if msg.encoding in ("rgb8", "RGB8"):
                img = PILImage.frombytes("RGB", (msg.width, msg.height), bytes(msg.data))
            elif msg.encoding in ("bgr8", "BGR8"):
                img = PILImage.frombytes("RGB", (msg.width, msg.height), bytes(msg.data))
                r, g, b = img.split()
                img = PILImage.merge("RGB", (b, g, r))
            elif msg.encoding in ("mono8", "8UC1"):
                img = PILImage.frombytes("L", (msg.width, msg.height), bytes(msg.data))
            else:
                return None
            buf = io.BytesIO()
            img.save(buf, format="JPEG", quality=85)
            return buf.getvalue()
        except ImportError:
            pass
        # Fallback: save as PPM
        if msg.encoding not in ("rgb8", "RGB8"):
            return None
        header = f"P6\n{msg.width} {msg.height}\n255\n".encode()
        return header + bytes(msg.data)
    except Exception as e:
        print(f"[CAM] Error convirtiendo imagen: {e}", flush=True)
        return None


class CameraCapture(Node):
    def __init__(self, output_dir: Path, interval_sec: float):
        super().__init__("camera_capture")
        self._output_dir = output_dir
        self._interval_sec = interval_sec
        self._lock = threading.Lock()
        self._latest: dict[str, tuple[float, Image]] = {}
        self._frame_count: dict[str, int] = {}
        self._last_saved: dict[str, float] = {}

        topics = [
            "/camera_overhead/image",
            "/camera_north/image",
            "/camera_south/image",
            "/camera_debug_top/image",
        ]
        for topic in topics:
            self.create_subscription(Image, topic, self._make_cb(topic), 5)
            self._frame_count[topic] = 0
            self._last_saved[topic] = 0.0

        # Timer for periodic saves
        self.create_timer(self._interval_sec, self._periodic_save)
        print(f"[CAM] Iniciado. Output={output_dir} interval={interval_sec}s", flush=True)
        print(f"[CAM] Suscrito a {len(topics)} tópicos de cámara", flush=True)

    def _make_cb(self, topic: str):
        def cb(msg: Image):
            with self._lock:
                self._latest[topic] = (time.time(), msg)
                self._frame_count[topic] += 1
                if self._frame_count[topic] == 1:
                    print(f"[CAM] Primer frame recibido: {topic} {msg.width}x{msg.height} {msg.encoding}", flush=True)
        return cb

    def _periodic_save(self):
        now = time.time()
        with self._lock:
            for topic, (ts, msg) in self._latest.items():
                if now - self._last_saved.get(topic, 0.0) < self._interval_sec * 0.9:
                    continue
                self._save_frame(topic, msg, now, "periodic")

    def save_event_frame(self, event_name: str):
        """Llama externamente para guardar frame en un evento (grasp, lift, etc.)."""
        now = time.time()
        with self._lock:
            for topic, (ts, msg) in self._latest.items():
                self._save_frame(topic, msg, now, event_name)

    def _save_frame(self, topic: str, msg: Image, ts: float, tag: str):
        if msg.width == 0 or msg.height == 0:
            return
        cam_name = topic.split("/")[1] if "/" in topic else "cam"
        ts_str = time.strftime("%H%M%S", time.localtime(ts))
        ext = ".jpg" if hasattr(msg, "encoding") and "mono" not in msg.encoding else ".jpg"
        fname = f"{ts_str}_{cam_name}_{tag}.jpg"
        fpath = self._output_dir / fname
        data = _ros_image_to_jpeg_bytes(msg)
        if data is None:
            return
        try:
            with open(fpath, "wb") as f:
                f.write(data)
            self._last_saved[topic] = ts
            print(f"[CAM] Guardado {fname} ({msg.width}x{msg.height} {msg.encoding})", flush=True)
        except Exception as e:
            print(f"[CAM] Error guardando {fname}: {e}", flush=True)

    def status_summary(self) -> str:
        with self._lock:
            parts = []
            for topic, count in self._frame_count.items():
                cam = topic.split("/")[1] if "/" in topic else topic
                latest = self._latest.get(topic)
                if latest:
                    ts, msg = latest
                    parts.append(f"{cam}:{count}frames {msg.width}x{msg.height}")
                else:
                    parts.append(f"{cam}:0frames")
            return " | ".join(parts)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True, help="Directorio de salida para los frames")
    parser.add_argument("--interval", type=float, default=2.0, help="Intervalo entre capturas en segundos")
    parser.add_argument("--timeout", type=float, default=700.0, help="Tiempo máximo de ejecución en segundos")
    args = parser.parse_args()

    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = CameraCapture(output_dir, args.interval)

    deadline = time.time() + args.timeout
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    print(f"[CAM] Ejecutando hasta {args.timeout:.0f}s...", flush=True)
    try:
        while rclpy.ok() and time.time() < deadline:
            executor.spin_once(timeout_sec=1.0)
    except KeyboardInterrupt:
        pass
    finally:
        summary = node.status_summary()
        print(f"[CAM] Fin. Resumen: {summary}", flush=True)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
