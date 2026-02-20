# RB_Humanoid_Control — Stage1 Validation Pack (Stand-Only)

## 0) 한 줄 요약
Stand-only 환경에서 `zero` baseline을 재현 가능하게 고정했고, termination/failure mode를 로그/요약/영상으로 추적 가능한 형태로 정리했다.

## 1) 목표와 범위
- 목표:
  - uncontrolled baseline(`zero`) 확보
  - pose-hold 비교 실험을 위한 기준점 고정
  - termination/failure mode 정량 기록 체계 구축
- 범위:
  - stand-only (`velocity_commands=0`)
  - `num_envs=1`, `steps=200`
- 비범위:
  - RL training (PPO 등)
  - 보행/조작 태스크 확장
  - PREEMPT_RT 실시간 튜닝

## 2) 시스템 아키텍처(제품 관점)
```
[Sensors/Obs] -> [State/Info Parse] -> [Controller Action] -> [Robot/Physics] -> [Safety] -> [Report]
IMU/Joint/Contact    done reason parsing     zero/pose action      IsaacLab G1      base_contact    summary/videos
```

## 3) Inputs / Outputs
### Inputs
- Base orientation / angular velocity
- Joint position / velocity
- Contact 및 termination signal
- Stand-only command (`velocity=0`)

### Outputs
- Action vector (`zero` 또는 `pose`)
- Rollout 통계 (`executed_steps`, `first_done_*`, `avg_reward_per_step`)

## 4) Safety / Failure Mode
- Termination 기준: `base_contact` (torso_link ground contact)
- 해석: torso 접촉 시 stand 실패로 판정

## 5) 실험 설계(재현 가능성)
- Task IDs:
  - `RB-Stage1-G1-Flat-Stand-v0`
  - `RB-Stage1-G1-Rough-Stand-v0`
- Entry point: `scripts/stage1/run_stage1.py`
- 자동 산출물:
  - raw: `logs/stage1/raw/<run_id>.log`
  - summary: `logs/stage1/summary/<run_id>_summary.md`
  - overview: `reports/stage1/overview.md`
  - videos: `logs/stage1/videos/*.mp4`

## 6) 결과 요약(현재 기준)
| Terrain | Mode | first_done_step | first_done_reason |
|---|---|---:|---|
| Flat | zero | 67 | base_contact |
| Rough | zero | 67 | base_contact |
| Flat | pose | pending | pending |
| Rough | pose | pending | pending |

## 7) 핵심 아티팩트
- Overview: `reports/stage1/overview.md`
- Summary:
  - `logs/stage1/summary/flat_zero_s200_20260220-121311_summary.md`
  - `logs/stage1/summary/rough_zero_s200_20260220-121937_summary.md`
- Videos:
  - `logs/stage1/videos/flat_zero_trim.mp4`
  - `logs/stage1/videos/rough_zero_trim.mp4`
- Images:
  - `reports/stage1/images/flat_zero.png`
  - `reports/stage1/images/rough_zero.png`

## 8) Roadmap
- Stage2: ROS2/C++ 노드 분리 + rosbag replay 회귀
- Stage3: PREEMPT_RT jitter 계측 + 제어 안정성 영향 비교

## 9) Reproduce
```bash
python scripts/stage1/run_stage1.py --task flat --mode zero --steps 200
python scripts/stage1/run_stage1.py --task rough --mode zero --steps 200
python scripts/stage1/run_stage1.py --task flat --mode pose --steps 200
python scripts/stage1/run_stage1.py --task rough --mode pose --steps 200
```
