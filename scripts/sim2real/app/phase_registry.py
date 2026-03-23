"""Phase registry for Isaac-side Sim2Real execution."""

from __future__ import annotations

from typing import Any, Callable

from .phase_handlers import (
    run_m1_sensor_phase,
    run_m3_command_phase,
    run_m5_pose_audit_phase,
    run_m5_stand_phase,
    run_m8_disturb_phase,
)

PhaseHandler = Callable[[Any, dict[str, Any], Any], dict[str, Any]]

PHASE_HANDLERS: dict[str, PhaseHandler] = {
    "m1_sensor": run_m1_sensor_phase,
    "m3_command": run_m3_command_phase,
    "m5_stand": run_m5_stand_phase,
    "m8_disturb": run_m8_disturb_phase,
    "m5_pose_audit": run_m5_pose_audit_phase,
}

SUPPORTED_PHASES = tuple(PHASE_HANDLERS.keys())


def get_phase_handler(phase: str) -> PhaseHandler:
    """Resolve a phase name into its handler."""
    try:
        return PHASE_HANDLERS[phase]
    except KeyError as exc:
        raise ValueError(f"Unsupported phase: {phase}") from exc


def run_phase(
    phase: str,
    *,
    args_cli: Any,
    phase_cfg: dict[str, Any],
    simulation_app: Any,
) -> dict[str, Any]:
    """Run the registered handler for the selected phase."""
    handler = get_phase_handler(phase)
    return handler(args_cli, phase_cfg, simulation_app)
