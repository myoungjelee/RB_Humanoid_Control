"""Compatibility facade for Isaac phase execution helpers.

New code should prefer:
- `phase_runtime.py` for runtime preparation and the common rollout runner
- `phase_handlers.py` for phase-specific wrappers
- `phase_registry.py` for phase-name dispatch
"""

from __future__ import annotations

from .phase_handlers import (
    run_m1_sensor_phase,
    run_m3_command_phase,
    run_m5_pose_audit_phase,
    run_m5_stand_phase,
    run_m8_disturb_phase,
)
from .phase_registry import PHASE_HANDLERS, SUPPORTED_PHASES, get_phase_handler, run_phase
from .phase_runtime import prepare_phase_runtime, run_rollout_phase

__all__ = [
    "PHASE_HANDLERS",
    "SUPPORTED_PHASES",
    "get_phase_handler",
    "prepare_phase_runtime",
    "run_m1_sensor_phase",
    "run_m3_command_phase",
    "run_m5_pose_audit_phase",
    "run_m5_stand_phase",
    "run_m8_disturb_phase",
    "run_phase",
    "run_rollout_phase",
]
