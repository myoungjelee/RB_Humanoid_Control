# M8 Result

- run_id: 20260315-000113
- verdict: PASS
- disturbance: torso impulse 113N x 0.10s

## Outcome
| label | result |
|---|---|
| balance_off | FALL at 6.435s |
| balance_on | NO_FALL_EVENT |

Interpretation: ON survived while OFF fell.

## Key KPI
| metric | off | on |
|---|---:|---:|
| peak_abs_tilt_r_after_disturb | 0.082 | 0.046 |
| peak_abs_tilt_p_after_disturb | 0.115 | 0.101 |

## Config
| field | value |
|---|---|
| tilt_qref_bias_abs_max | 0.03 |
| hip_pitch_joint_trim | +0.005 |
| ankle_pitch_joint_trim | +0.005 |
