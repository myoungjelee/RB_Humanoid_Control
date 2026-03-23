"""Standalone Isaac world runtime hooks."""

from __future__ import annotations

import math
import os
from pathlib import Path
from typing import Any

from .telemetry import (
    log_config,
    log_disturbance_end,
    log_disturbance_start,
    log_disturbance_sync,
    log_fall_event,
    log_warning,
)


def _canonicalize_disturbance_frame_mode(mode: str) -> str:
    """외란 입력 force/torque를 어떤 기준축으로 해석할지 정규화한다."""
    value = str(mode).strip().lower()
    if value in {"identity", "body_local", "torso_local"}:
        return "identity"
    if value in {"g1_imu_link", "control_frame"}:
        return "g1_imu_link"
    return "identity"


def _remap_disturbance_wrench(
    force_xyz: tuple[float, float, float],
    torque_xyz: tuple[float, float, float],
    frame_mode: str,
) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
    """설정된 control frame wrench를 실제 torso local frame wrench로 변환한다."""
    if frame_mode != "g1_imu_link":
        return force_xyz, torque_xyz

    fx, fy, fz = force_xyz
    tx, ty, tz = torque_xyz
    mapped_force = (fy, fx, fz)
    mapped_torque = (ty, tx, tz)
    return mapped_force, mapped_torque


class StandaloneDirectWorld:
    """IsaacLab direct spawn 기반 standalone simulation 래퍼."""

    def __init__(
        self,
        sim: Any,
        scene: Any,
        robot: Any,
        asset_source: str,
        sim_dt: float,
        fall_watch_cfg: dict[str, Any] | None = None,
        disturbance_cfg: dict[str, Any] | None = None,
    ):
        self.world = sim
        self.scene = scene
        self.robot = robot
        self.asset_source = asset_source
        self.sim_dt = float(sim_dt)
        self._step_count = 0
        self._fall_watch_cfg = dict(fall_watch_cfg or {})
        self._fall_watch_enabled = bool(self._fall_watch_cfg.get("enabled", False))
        self._fall_event_emitted = False
        self._fall_watch_min_elapsed_sec = float(self._fall_watch_cfg.get("min_elapsed_sec", 0.5))
        self._fall_watch_pelvis_z_threshold = float(self._fall_watch_cfg.get("pelvis_z_threshold", 0.45))
        self._fall_watch_torso_z_threshold = float(self._fall_watch_cfg.get("torso_z_threshold", 0.58))
        torso_name_candidates = self._fall_watch_cfg.get(
            "torso_body_name_candidates", ["torso_link", "torso", "base_link"]
        )
        self._torso_body_index, self._torso_body_name = self._resolve_body_candidate(torso_name_candidates)
        self._disturbance_cfg = dict(disturbance_cfg or {})
        self._disturbance_enabled = bool(self._disturbance_cfg.get("enabled", False))
        self._disturbance_active = False
        self._disturbance_completed = False
        self._disturbance_start_elapsed_sec = float(self._disturbance_cfg.get("start_elapsed_sec", 5.0))
        self._disturbance_start_after_control_active_sec = float(
            self._disturbance_cfg.get("start_after_control_active_sec", -1.0)
        )
        self._disturbance_duration_sec = float(self._disturbance_cfg.get("duration_sec", 0.2))
        self._disturbance_end_elapsed_sec = self._disturbance_start_elapsed_sec + self._disturbance_duration_sec
        self._disturbance_sync_marker_path = str(os.environ.get("DISTURBANCE_SYNC_MARKER", "")).strip()
        self._disturbance_control_active_elapsed_sec: float | None = None
        self._disturbance_is_global = bool(self._disturbance_cfg.get("is_global", True))
        self._disturbance_frame_mode = _canonicalize_disturbance_frame_mode(
            str(self._disturbance_cfg.get("frame_mode", "identity"))
        )
        force_xyz = self._disturbance_cfg.get("force_xyz", [0.0, 0.0, 0.0])
        torque_xyz = self._disturbance_cfg.get("torque_xyz", [0.0, 0.0, 0.0])
        self._disturbance_force_xyz = tuple(float(v) for v in force_xyz)
        self._disturbance_torque_xyz = tuple(float(v) for v in torque_xyz)
        (
            self._disturbance_applied_force_xyz,
            self._disturbance_applied_torque_xyz,
        ) = _remap_disturbance_wrench(
            self._disturbance_force_xyz,
            self._disturbance_torque_xyz,
            self._disturbance_frame_mode,
        )
        disturbance_body_candidates = self._disturbance_cfg.get(
            "body_name_candidates", ["torso_link", "torso", "base_link"]
        )
        self._disturbance_body_index, self._disturbance_body_name = self._resolve_body_candidate(
            disturbance_body_candidates
        )
        if self._fall_watch_enabled:
            torso_label = self._torso_body_name if self._torso_body_name is not None else "unresolved"
            log_config(
                "fall_watch enabled "
                f"min_elapsed_sec={self._fall_watch_min_elapsed_sec:.2f} "
                f"pelvis_z_threshold={self._fall_watch_pelvis_z_threshold:.3f} "
                f"torso_z_threshold={self._fall_watch_torso_z_threshold:.3f} "
                f"torso_body={torso_label}"
            )
        if self._disturbance_enabled:
            body_label = self._disturbance_body_name if self._disturbance_body_name is not None else "unresolved"
            log_config(
                "disturbance enabled "
                f"start_elapsed_sec={self._disturbance_start_elapsed_sec:.2f} "
                f"start_after_control_active_sec={self._disturbance_start_after_control_active_sec:.2f} "
                f"duration_sec={self._disturbance_duration_sec:.2f} "
                f"body={body_label} "
                f"frame_mode={self._disturbance_frame_mode} "
                f"configured_force_xyz={self._disturbance_force_xyz} "
                f"configured_torque_xyz={self._disturbance_torque_xyz} "
                f"applied_force_xyz={self._disturbance_applied_force_xyz} "
                f"applied_torque_xyz={self._disturbance_applied_torque_xyz} "
                f"is_global={self._disturbance_is_global} "
                f"sync_marker={self._disturbance_sync_marker_path or 'disabled'}"
            )

    def step(self, *, render: bool = True) -> None:
        """physics/render step을 1회 진행한다."""
        self._maybe_apply_disturbance()
        self.world.step(render=render)
        self.scene.update(self.sim_dt)
        self._step_count += 1

    def _resolve_body_candidate(self, name_candidates: Any) -> tuple[int | None, str | None]:
        if not name_candidates:
            return None, None
        if isinstance(name_candidates, str):
            candidate_list = [name_candidates]
        else:
            candidate_list = [str(name) for name in name_candidates]
        for candidate in candidate_list:
            body_ids, body_names = self.robot.find_bodies(candidate, preserve_order=True)
            if body_ids:
                return int(body_ids[0]), str(body_names[0])
        log_warning(f"fall_watch body candidates not found: {candidate_list}")
        return None, None

    def _apply_disturbance_wrench(self, *, active: bool) -> None:
        """disturbance wrench를 직접 PhysX view에 적용한다."""
        if self._disturbance_body_index is None:
            return
        body_count = len(getattr(self.robot, "body_names"))
        forces = self.robot.data.root_pos_w.new_zeros((1, body_count, 3))
        torques = self.robot.data.root_pos_w.new_zeros((1, body_count, 3))
        if active:
            fx, fy, fz = self._disturbance_applied_force_xyz
            tx, ty, tz = self._disturbance_applied_torque_xyz
            forces[:, self._disturbance_body_index, 0] = fx
            forces[:, self._disturbance_body_index, 1] = fy
            forces[:, self._disturbance_body_index, 2] = fz
            torques[:, self._disturbance_body_index, 0] = tx
            torques[:, self._disturbance_body_index, 1] = ty
            torques[:, self._disturbance_body_index, 2] = tz
        self.robot.root_physx_view.apply_forces_and_torques_at_position(
            force_data=forces.view(-1, 3),
            torque_data=torques.view(-1, 3),
            position_data=None,
            indices=self.robot._ALL_INDICES,
            is_global=self._disturbance_is_global,
        )

    def _maybe_apply_disturbance(self) -> None:
        """설정된 외란을 일정 시간 창에서만 적용한다."""
        if not self._disturbance_enabled or self._disturbance_body_index is None or self._disturbance_completed:
            return
        elapsed_sec = self._step_count * self.sim_dt
        if (
            self._disturbance_control_active_elapsed_sec is None
            and self._disturbance_sync_marker_path
            and Path(self._disturbance_sync_marker_path).exists()
        ):
            self._disturbance_control_active_elapsed_sec = elapsed_sec
            log_disturbance_sync(
                elapsed_sec=elapsed_sec,
                marker=self._disturbance_sync_marker_path,
            )

        active_window_start = self._disturbance_start_elapsed_sec
        if (
            self._disturbance_start_after_control_active_sec >= 0.0
            and self._disturbance_control_active_elapsed_sec is not None
        ):
            active_window_start = (
                self._disturbance_control_active_elapsed_sec
                + self._disturbance_start_after_control_active_sec
            )
        active_window_end = active_window_start + self._disturbance_duration_sec

        if active_window_start <= elapsed_sec < active_window_end:
            if not self._disturbance_active:
                log_disturbance_start(
                    elapsed_sec=elapsed_sec,
                    window_start_sec=active_window_start,
                    body=self._disturbance_body_name,
                    frame_mode=self._disturbance_frame_mode,
                    configured_force_xyz=self._disturbance_force_xyz,
                    configured_torque_xyz=self._disturbance_torque_xyz,
                    applied_force_xyz=self._disturbance_applied_force_xyz,
                    applied_torque_xyz=self._disturbance_applied_torque_xyz,
                    is_global=self._disturbance_is_global,
                )
                self._disturbance_active = True
            self._apply_disturbance_wrench(active=True)
            return
        if self._disturbance_active:
            self._apply_disturbance_wrench(active=False)
            log_disturbance_end(
                elapsed_sec=elapsed_sec,
                window_end_sec=active_window_end,
                body=self._disturbance_body_name,
            )
            self._disturbance_active = False
            self._disturbance_completed = True

    def maybe_emit_fall_event(self) -> bool:
        """pelvis/torso 높이 하강을 fall event로 간주하고 한 번만 로그를 남긴다."""
        if not self._fall_watch_enabled or self._fall_event_emitted:
            return False
        elapsed_sec = self._step_count * self.sim_dt
        if elapsed_sec < self._fall_watch_min_elapsed_sec:
            return False

        env_origin_z = 0.0
        env_origins = getattr(self.scene, "env_origins", None)
        if env_origins is not None:
            env_origin_z = float(env_origins[0, 2].item())

        pelvis_z = float(self.robot.data.root_pos_w[0, 2].item()) - env_origin_z
        torso_z = math.nan
        if self._torso_body_index is not None:
            torso_z = float(self.robot.data.body_pos_w[0, self._torso_body_index, 2].item()) - env_origin_z

        pelvis_trigger = pelvis_z <= self._fall_watch_pelvis_z_threshold
        torso_trigger = not math.isnan(torso_z) and torso_z <= self._fall_watch_torso_z_threshold
        if not pelvis_trigger and not torso_trigger:
            return False

        trigger_parts: list[str] = []
        if pelvis_trigger:
            trigger_parts.append("pelvis")
        if torso_trigger:
            torso_name = self._torso_body_name or "torso"
            trigger_parts.append(f"torso:{torso_name}")

        torso_z_text = "nan" if math.isnan(torso_z) else f"{torso_z:.3f}"
        log_fall_event(
            elapsed_sec=elapsed_sec,
            trigger='+'.join(trigger_parts),
            pelvis_z=pelvis_z,
            pelvis_threshold=self._fall_watch_pelvis_z_threshold,
            torso_z_text=torso_z_text,
            torso_threshold=self._fall_watch_torso_z_threshold,
        )
        self._fall_event_emitted = True
        return True

    def close(self) -> None:
        """명시적으로 정리할 리소스가 있으면 정리한다."""
        stop = getattr(self.world, "stop", None)
        if callable(stop):
            stop()
