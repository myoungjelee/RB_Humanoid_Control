# RB_Humanoid_Control

Humanoid / Legged Robot Control Validation Project
Isaac Sim + IsaacLab 기반 휴머노이드 제어 시스템 검증 프로젝트

## Project Goal

본 프로젝트는 휴머노이드 로봇의 제어 성능 자체를 과시하기보다,

**센서 -> 상태 -> 명령 -> 액션 -> 종료 조건**

전체 실행 경로를 재현 및 검증하는 것을 목표로 합니다.

강화학습 성능 경쟁이 아니라, 시스템 검증 능력과 실험 설계 역량을
재현 가능한 로그/요약/리포트로 증명하는 데 초점을 둡니다.

## Stage 1: Stand-Only Validation

### 목적

- velocity command를 0으로 고정한 stand-only 환경에서 실행 안정성을 검증
- zero-action baseline을 통해 실패 시점을 확인
- pose-hold baseline 비교를 위한 기준선 확보

### 실험 구성

- task
  - `RB-Stage1-G1-Flat-Stand-v0`
  - `RB-Stage1-G1-Rough-Stand-v0`
- `num_envs=1`
- `steps=200`
- device: `cuda`
- command: `velocity=[0, 0, 0]`

### Stage1-1: Validity Check

- `velocity_commands == [0.0, 0.0, 0.0]` 확인
- `reset/step` 루프 정상 동작 확인
- termination 처리 동작 확인
  - 기본값: 첫 done에서 종료 (`--reset_on_done` 미사용)
  - 옵션: done마다 reset 후 계속 (`--reset_on_done`)

### Stage1-2: Zero-Action Baseline (현재 결과)

- action: `0`
- 목적: 관절 제어 없이 시스템이 언제 종료되는지 측정

| Task | Mode | first_done_step | first_done_reason | status |
| --- | --- | --- | --- | --- |
| Flat | zero | 67 | unknown | stopped_on_first_done |
| Rough | zero | 67 | unknown | stopped_on_first_done |

해석:
- 제어가 없으면 stand-only 환경에서 빠르게 종료됨
- 실행 경로(명령 입력/step/termination)는 정상적으로 작동함
- done 원인은 현재 `unknown`으로 기록되어 원인 세분화가 다음 과제임

## Reproduction

```bash
python scripts/stage1/run_stage1.py --task flat --mode zero --num_envs 1 --steps 200
python scripts/stage1/run_stage1.py --task rough --mode zero --num_envs 1 --steps 200
```

Artifacts:
- Raw log: `logs/stage1/raw/`
- Summary: `logs/stage1/summary/`
- Human report: `reports/stage1/overview.md`

## Next Step

- pose-hold baseline 추가 및 zero 대비 비교
- termination reason 정밀 분석
- 외란(초기 자세 랜덤, small disturbance) 검증

## Tech Stack

- Isaac Sim 5.x
- IsaacLab (source build)
- PyTorch (CUDA)
- Gymnasium

## Project Structure

```text
scripts/
  stage1/
    run_stage1.py

rb_utils/
  __init__.py
  action_generators.py
  experiment.py
  experiment_runner.py
  summary_writer.py
  termination_utils.py

logs/
  stage1/
    raw/
    summary/

reports/
  stage1/
    overview.md
```

## Note

본 프로젝트는 Stage 1 기준으로 시스템 검증 중심입니다.
RL 학습, ROS2 통합, locomotion 확장은 Stage 1 범위 밖으로 분리해 관리합니다.
