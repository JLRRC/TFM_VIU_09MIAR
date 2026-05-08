#!/usr/bin/env python3
"""
attach_gate_evaluator.py
Evaluador temporal del gate de attach para el pipeline de pick.

Reemplaza la comprobación de proximidad simple de _wait_demo_attach_follow
con validación multi-fuente y ventana temporal:
  - ventana deslizante configurable de muestras (tcp, objeto)
  - cálculo de drift de la pose relativa objeto-TCP
  - verificación de gripper cerrado con apertura medida
  - separación explícita de fuentes de verdad (tf_world, tf_base, visual, fk, command)
  - logs estructurados [ATTACH_GATE][CHECK|PASS|FAIL|WARN]
"""
from __future__ import annotations

import math
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Callable, Deque, List, Optional, Tuple

Vec3 = Tuple[float, float, float]


@dataclass
class PoseSample:
    """Una muestra temporal de pose TCP y objeto."""
    timestamp: float
    tcp_tf_world: Optional[Vec3]   # world → rg2_pinch_center (TF live)
    tcp_tf_base: Optional[Vec3]    # base_link → rg2_pinch_center (TF live)
    object_world: Optional[Vec3]   # pose real del objeto en world
    object_base: Optional[Vec3]    # pose real del objeto en base_link
    gripper_opening_m: Optional[float] = None

    @property
    def rel_vec_base(self) -> Optional[Vec3]:
        """Vector relativo objeto-TCP en base_link (object - tcp)."""
        if self.object_base is None or self.tcp_tf_base is None:
            return None
        return (
            self.object_base[0] - self.tcp_tf_base[0],
            self.object_base[1] - self.tcp_tf_base[1],
            self.object_base[2] - self.tcp_tf_base[2],
        )


@dataclass
class AttachGateConfig:
    """Parámetros configurables del evaluador de ATTACH_GATE."""

    # Distancia máxima TCP ↔ objeto para considerar proximidad
    max_tcp_obj_dist_m: float = 0.040
    # Drift máximo de la pose relativa objeto-TCP a lo largo de la ventana
    max_rel_drift_m: float = 0.012
    # Diferencia máxima aceptable entre tcp_tf y tcp_visual (solo warning)
    max_tf_visual_gap_m: float = 0.020
    # Duración de la ventana temporal de muestras
    stable_window_sec: float = 0.35
    # Número mínimo de muestras en la ventana para aprobar
    min_stable_samples: int = 5
    # Apertura máxima del gripper para considerarlo cerrado (suma de joints, m)
    gripper_closed_threshold_m: float = 0.020
    # Intervalo entre muestras (s)
    sample_interval_sec: float = 0.06
    # Tiempo máximo total de espera
    timeout_sec: float = 2.0


@dataclass
class AttachGateResult:
    """Resultado detallado de la evaluación de ATTACH_GATE."""

    ok: bool = False
    reason: str = "not_evaluated"
    # Última muestra
    tcp_tf_world: Optional[Vec3] = None
    tcp_tf_base: Optional[Vec3] = None
    tcp_fk_base: Optional[Vec3] = None
    tcp_command_base: Optional[Vec3] = None
    object_world: Optional[Vec3] = None
    object_base: Optional[Vec3] = None
    gripper_opening_m: Optional[float] = None
    gripper_closed: bool = False
    attach_backend_ok: bool = False
    # Métricas calculadas
    tcp_obj_dist: Optional[float] = None
    rel_drift: Optional[float] = None
    tf_visual_gap: Optional[float] = None
    stable_samples: int = 0
    # Warnings acumulados
    warnings: List[str] = field(default_factory=list)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _dist3(a: Optional[Vec3], b: Optional[Vec3]) -> Optional[float]:
    if a is None or b is None:
        return None
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


def _fmt3(v: Optional[Vec3], dec: int = 3) -> str:
    if v is None:
        return "(none)"
    return f"({v[0]:.{dec}f},{v[1]:.{dec}f},{v[2]:.{dec}f})"


def _fmts(v: Optional[float], dec: int = 4) -> str:
    return "none" if v is None else f"{v:.{dec}f}"


# ---------------------------------------------------------------------------
# Evaluator
# ---------------------------------------------------------------------------

class AttachGateEvaluator:
    """
    Evaluador de la fase ATTACH_GATE con ventana temporal.

    Uso típico en el worker del pipeline::

        from ur5_qt_panel.attach_gate_evaluator import AttachGateEvaluator, AttachGateConfig

        cfg = AttachGateConfig(
            max_tcp_obj_dist_m=attach_follow_max_tcp_dist_m,
            max_rel_drift_m=0.012,
            stable_window_sec=0.35,
            min_stable_samples=5,
            timeout_sec=attach_follow_timeout_sec,
        )
        ev = AttachGateEvaluator(
            config=cfg,
            emit_log=panel._emit_log,
            tcp_tf_world_fn=_live_tcp_world,
            tcp_tf_base_fn=_live_tcp_base,
            object_world_fn=_live_object_world,
            object_base_fn=_live_object_base,
            gripper_opening_fn=lambda: (_read_gripper_state() or {}).get("opening_sum"),
            attach_backend_ok=attach_ok,
            tcp_command_base=tcp_base_grasp,
        )
        result = ev.evaluate()
        if not result.ok:
            raise RuntimeError(
                f"demo_attach_follow_not_confirmed reason={result.reason} "
                f"tcp_obj_dist={_fmts(result.tcp_obj_dist)} "
                f"rel_drift={_fmts(result.rel_drift)} "
                f"stable_samples={result.stable_samples}"
            )
    """

    def __init__(
        self,
        *,
        config: AttachGateConfig,
        emit_log: Callable[[str], None],
        tcp_tf_world_fn: Callable[[], Optional[Vec3]],
        tcp_tf_base_fn: Callable[[], Optional[Vec3]],
        object_world_fn: Callable[[], Optional[Vec3]],
        object_base_fn: Callable[[], Optional[Vec3]],
        gripper_opening_fn: Optional[Callable[[], Optional[float]]] = None,
        attach_backend_ok: bool = False,
        tcp_fk_base: Optional[Vec3] = None,
        tcp_command_base: Optional[Vec3] = None,
        tcp_visual_world: Optional[Vec3] = None,
        object_pose_source: str = "live_object_world",
    ) -> None:
        self._cfg = config
        self._log = emit_log
        self._tcp_tf_world_fn = tcp_tf_world_fn
        self._tcp_tf_base_fn = tcp_tf_base_fn
        self._object_world_fn = object_world_fn
        self._object_base_fn = object_base_fn
        self._gripper_opening_fn = gripper_opening_fn
        self._attach_backend_ok = attach_backend_ok
        self._tcp_fk_base = tcp_fk_base
        self._tcp_command_base = tcp_command_base
        self._tcp_visual_world = tcp_visual_world
        self._object_pose_source = str(object_pose_source or "live_object_world")
        # When strict, object_world=None is treated as a source failure, not a missing sample.
        self._strict_pose_source = (self._object_pose_source == "strict_fresh_gazebo")
        self._got_live_pose_sample = False
        self._window: Deque[PoseSample] = deque()

    # ------------------------------------------------------------------

    def _sample(self) -> PoseSample:
        opening: Optional[float] = None
        if self._gripper_opening_fn is not None:
            try:
                opening = self._gripper_opening_fn()
            except Exception:
                pass
        return PoseSample(
            timestamp=time.time(),
            tcp_tf_world=self._tcp_tf_world_fn(),
            tcp_tf_base=self._tcp_tf_base_fn(),
            object_world=self._object_world_fn(),
            object_base=self._object_base_fn(),
            gripper_opening_m=opening,
        )

    def _prune(self, now: float) -> None:
        cutoff = now - self._cfg.stable_window_sec
        while self._window and self._window[0].timestamp < cutoff:
            self._window.popleft()

    def _drift(self) -> Optional[float]:
        """Drift máximo de la pose relativa objeto-TCP en la ventana."""
        valid = [s for s in self._window if s.rel_vec_base is not None]
        if len(valid) < 2:
            return None
        ref = valid[0].rel_vec_base
        if ref is None:
            return None
        max_drift = 0.0
        for s in valid[1:]:
            cur = s.rel_vec_base
            if cur is None:
                continue
            d = math.sqrt(sum((cur[i] - ref[i]) ** 2 for i in range(3)))
            if d > max_drift:
                max_drift = d
        return max_drift

    def _log_check(
        self,
        sample: PoseSample,
        tcp_obj_dist: Optional[float],
        drift: Optional[float],
        tf_visual_gap: Optional[float],
        gripper_closed: bool,
        reason: str,
    ) -> None:
        self._log(
            "[ATTACH_GATE][CHECK] "
            f"tcp_tf_world={_fmt3(sample.tcp_tf_world)} "
            f"tcp_tf_base={_fmt3(sample.tcp_tf_base)} "
            f"tcp_fk_base={_fmt3(self._tcp_fk_base)} "
            f"tcp_command_base={_fmt3(self._tcp_command_base)} "
            f"object_world={_fmt3(sample.object_world)} "
            f"object_base={_fmt3(sample.object_base)} "
            f"attach_gate_live_pose_source={self._object_pose_source} "
            f"tcp_obj_dist={_fmts(tcp_obj_dist)} "
            f"rel_drift={_fmts(drift)} "
            f"tf_visual_gap={_fmts(tf_visual_gap)} "
            f"gripper_opening_m={_fmts(sample.gripper_opening_m)} "
            f"gripper_closed={str(gripper_closed).lower()} "
            f"attach_backend_ok={str(self._attach_backend_ok).lower()} "
            f"stable_samples={len(self._window)} "
            f"reason={reason} "
            f"ok=false"
        )

    # ------------------------------------------------------------------

    def evaluate(self) -> AttachGateResult:
        cfg = self._cfg
        result = AttachGateResult(
            attach_backend_ok=self._attach_backend_ok,
            tcp_fk_base=self._tcp_fk_base,
            tcp_command_base=self._tcp_command_base,
        )

        if not self._attach_backend_ok:
            self._log("[ATTACH_GATE][FAIL] reason=attach_backend_not_ok ok=false")
            result.reason = "attach_backend_not_ok"
            return result

        deadline = time.time() + cfg.timeout_sec
        last: Optional[PoseSample] = None
        next_log = 0.0

        while time.time() < deadline:
            s = self._sample()
            last = s

            # Strict source enforcement: if the caller requires a live Gazebo pose and
            # both object_world and object_base are None, the source has degraded (e.g.
            # pose_snapshot unavailable, entity not yet on pose/info after spawn).
            # Do NOT add this sample to the window and do NOT allow approval.
            if self._strict_pose_source and s.object_world is None and s.object_base is None:
                self._log(
                    "[ATTACH_GATE][WARN] degraded_pose_source "
                    f"attach_gate_live_pose_source={self._object_pose_source} "
                    "object_world=none object_base=none "
                    "strict_source=true sample_rejected=true"
                )
                time.sleep(cfg.sample_interval_sec)
                continue
            if s.object_world is not None or s.object_base is not None:
                self._got_live_pose_sample = True

            self._window.append(s)
            self._prune(s.timestamp)

            opening = s.gripper_opening_m
            gripper_closed = (
                opening is not None and opening <= cfg.gripper_closed_threshold_m
            )
            tcp_obj_dist = _dist3(s.tcp_tf_base, s.object_base)
            drift = self._drift()
            tf_visual_gap = _dist3(self._tcp_visual_world, s.tcp_tf_world)

            # --- Log periódico ---
            now = time.time()
            if now >= next_log:
                self._log_check(s, tcp_obj_dist, drift, tf_visual_gap, gripper_closed, "sampling")
                next_log = now + 0.25

            # --- Warning: tf_visual gap excesivo → no aprobar en este ciclo ---
            if tf_visual_gap is not None and tf_visual_gap > cfg.max_tf_visual_gap_m:
                wmsg = (
                    f"[ATTACH_GATE][WARN] tf_visual_mismatch "
                    f"gap={tf_visual_gap:.4f} threshold={cfg.max_tf_visual_gap_m:.4f} "
                    "blocking_approval=true"
                )
                self._log(wmsg)
                if wmsg not in result.warnings:
                    result.warnings.append(wmsg)
                time.sleep(cfg.sample_interval_sec)
                continue

            # --- Fallo: TCP demasiado lejos del objeto ---
            if tcp_obj_dist is not None and tcp_obj_dist > cfg.max_tcp_obj_dist_m:
                self._log(
                    f"[ATTACH_GATE][FAIL] reason=tcp_object_too_far "
                    f"tcp_obj_dist={tcp_obj_dist:.4f} max={cfg.max_tcp_obj_dist_m:.4f}"
                )
                result.tcp_obj_dist = tcp_obj_dist
                time.sleep(cfg.sample_interval_sec)
                continue

            # --- Esperar muestras suficientes ---
            if len(self._window) < cfg.min_stable_samples:
                self._log(
                    f"[ATTACH_GATE][WARN] insufficient_samples "
                    f"have={len(self._window)} need={cfg.min_stable_samples}"
                )
                time.sleep(cfg.sample_interval_sec)
                continue

            # --- Fallo: drift excesivo en ventana ---
            if drift is None:
                self._log(
                    "[ATTACH_GATE][WARN] insufficient_samples "
                    "reason=drift_not_computable rel_vec_base_unavailable=true"
                )
                time.sleep(cfg.sample_interval_sec)
                continue

            if drift > cfg.max_rel_drift_m:
                self._log(
                    f"[ATTACH_GATE][FAIL] reason=rel_drift_too_high "
                    f"drift={drift:.4f} max={cfg.max_rel_drift_m:.4f}"
                )
                result.rel_drift = drift
                time.sleep(cfg.sample_interval_sec)
                continue

            # --- Warning gripper (no bloquea) ---
            if opening is not None and not gripper_closed:
                wmsg = (
                    f"[ATTACH_GATE][WARN] gripper_not_closed_measured "
                    f"opening_m={opening:.4f} threshold={cfg.gripper_closed_threshold_m:.4f}"
                )
                self._log(wmsg)
                if wmsg not in result.warnings:
                    result.warnings.append(wmsg)

            # --- PASS ---
            result.ok = True
            result.reason = "ok"
            result.tcp_tf_world = s.tcp_tf_world
            result.tcp_tf_base = s.tcp_tf_base
            result.object_world = s.object_world
            result.object_base = s.object_base
            result.gripper_opening_m = opening
            result.gripper_closed = gripper_closed
            result.tcp_obj_dist = tcp_obj_dist
            result.rel_drift = drift
            result.tf_visual_gap = tf_visual_gap
            result.stable_samples = len(self._window)
            self._log(
                "[ATTACH_GATE][PASS] "
                f"tcp_tf_world={_fmt3(s.tcp_tf_world)} "
                f"tcp_tf_base={_fmt3(s.tcp_tf_base)} "
                f"tcp_fk_base={_fmt3(self._tcp_fk_base)} "
                f"tcp_command_base={_fmt3(self._tcp_command_base)} "
                f"object_world={_fmt3(s.object_world)} "
                f"object_base={_fmt3(s.object_base)} "
                f"attach_gate_live_pose_source={self._object_pose_source} "
                f"tcp_obj_dist={_fmts(tcp_obj_dist)} "
                f"rel_drift={_fmts(drift)} "
                f"tf_visual_gap={_fmts(tf_visual_gap)} "
                f"gripper_opening_m={_fmts(opening)} "
                f"gripper_closed={str(gripper_closed).lower()} "
                f"attach_backend_ok=true "
                f"stable_samples={len(self._window)} "
                f"reason=ok ok=true"
            )
            return result

        # --- Timeout ---
        if last is not None:
            result.tcp_tf_world = last.tcp_tf_world
            result.tcp_tf_base = last.tcp_tf_base
            result.object_world = last.object_world
            result.object_base = last.object_base
            result.gripper_opening_m = last.gripper_opening_m
            result.gripper_closed = (
                last.gripper_opening_m is not None
                and last.gripper_opening_m <= cfg.gripper_closed_threshold_m
            )
            result.stable_samples = len(self._window)

        result.tcp_obj_dist = _dist3(
            result.tcp_tf_base, result.object_base
        )
        result.rel_drift = self._drift()

        if self._strict_pose_source and not self._got_live_pose_sample:
            result.reason = "degraded_pose_source"
        elif result.tcp_obj_dist is not None and result.tcp_obj_dist > cfg.max_tcp_obj_dist_m:
            result.reason = "timeout_tcp_object_too_far"
        elif len(self._window) < cfg.min_stable_samples:
            result.reason = "timeout_insufficient_samples"
        elif result.rel_drift is not None and result.rel_drift > cfg.max_rel_drift_m:
            result.reason = "timeout_rel_drift_too_high"
        else:
            result.reason = "timeout"

        self._log(
            "[ATTACH_GATE][FAIL] "
            f"tcp_tf_world={_fmt3(result.tcp_tf_world)} "
            f"tcp_tf_base={_fmt3(result.tcp_tf_base)} "
            f"tcp_fk_base={_fmt3(self._tcp_fk_base)} "
            f"tcp_command_base={_fmt3(self._tcp_command_base)} "
            f"object_world={_fmt3(result.object_world)} "
            f"object_base={_fmt3(result.object_base)} "
            f"tcp_obj_dist={_fmts(result.tcp_obj_dist)} "
            f"rel_drift={_fmts(result.rel_drift)} "
            f"tf_visual_gap={_fmts(result.tf_visual_gap)} "
            f"gripper_opening_m={_fmts(result.gripper_opening_m)} "
            f"gripper_closed={str(result.gripper_closed).lower()} "
            f"attach_backend_ok={str(self._attach_backend_ok).lower()} "
            f"stable_samples={result.stable_samples} "
            f"reason={result.reason} ok=false"
        )
        return result
