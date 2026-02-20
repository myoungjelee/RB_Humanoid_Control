# RB_Humanoid_Control

Humanoid control validation portfolio with two tracks:
- Main: ROS2 Sim-to-Real (`ros2_ws/`)
- Baseline: IsaacLab Stage1 stand validation (`scripts/stage1/`)

## Project Direction

핵심 목표는 제어 단일 실험을 넘어서,
**센서 -> 추정 -> 제어 -> 안전 -> 로깅/리포트** 전체 경로를
Sim과 Real에서 재사용 가능한 인터페이스로 정리하는 것입니다.

- 메인트랙: ROS2 Sim-to-Real 아키텍처 구축
- 베이스라인: Stage1 실험 자산은 기준선으로 유지

## Track A: ROS2 Sim-to-Real (Main)

현재는 M0/M1(인터페이스 고정, 센서 토픽 파이프라인) 단계 기준으로 진행합니다.

- 목표 산출물
  - `reports/sim2real/overview.md`
  - `reports/sim2real/ONE_PAGER.md`
  - `logs/sim2real/<timestamp>/`
- 핵심 노드(계획)
  - `rb_state_estimator`
  - `rb_controller`
  - `rb_safety_monitor`
  - `rb_kpi_logger`

## Track B: Stage1 Baseline (Frozen Evidence)

Stage1은 stand-only 기준선을 고정해두는 트랙입니다.

- task
  - `RB-Stage1-G1-Flat-Stand-v0`
  - `RB-Stage1-G1-Rough-Stand-v0`
- 조건
  - `num_envs=1`
  - `steps=200`
  - velocity command = `[0, 0, 0]`

현재 baseline 결과(Zero mode):

| Terrain | Mode | first_done_step | first_done_reason |
| --- | --- | ---: | --- |
| Flat | zero | 67 | base_contact |
| Rough | zero | 67 | base_contact |

## Reproduction (Stage1 Baseline)

```bash
python scripts/stage1/run_stage1.py --task flat --mode zero --num_envs 1 --steps 200
python scripts/stage1/run_stage1.py --task rough --mode zero --num_envs 1 --steps 200
python scripts/stage1/run_stage1.py --task flat --mode pose --num_envs 1 --steps 200
python scripts/stage1/run_stage1.py --task rough --mode pose --num_envs 1 --steps 200
```

## Reports

- Sim-to-Real
  - `reports/sim2real/overview.md`
  - `reports/sim2real/ONE_PAGER.md`
- Stage1 Baseline
  - `reports/stage1/overview.md`
  - `reports/stage1/ONE_PAGER.md`

## Artifacts

- Stage1 raw log: `logs/stage1/raw/`
- Stage1 summary: `logs/stage1/summary/`
- Stage1 videos: `logs/stage1/videos/`
- Stage1 figures/images: `reports/stage1/figures/`, `reports/stage1/images/`
- Sim2Real raw/summary: `logs/sim2real/raw/`, `logs/sim2real/summary/`

## ROS2 Tracking Policy

- Track(커밋):
  - `ros2_ws/src/**` (노드/패키지 소스)
  - `launch/`, `config/`, `package.xml`, `CMakeLists.txt`, `setup.py`
  - `reports/sim2real/**` (요약 문서/그림)
- Untrack(커밋 안 함):
  - `ros2_ws/build/`, `ros2_ws/install/`, `ros2_ws/log/`
  - 대용량 실행 산출물(`logs/**`, rosbag raw 데이터)

원칙: 실행 산출물은 `logs/`에 남기고, 공유용 결과만 `reports/`로 정리한다.

## Tech Stack

- Isaac Sim 5.x
- IsaacLab (source build)
- ROS2 (Sim-to-Real track)
- PyTorch (CUDA)
- Gymnasium

## Project Structure

```text
scripts/
  stage1/
    run_stage1.py

rb_utils/
  action_generators.py
  experiment.py
  experiment_runner.py
  summary_writer.py
  termination_utils.py

logs/
  stage1/
    raw/
    summary/
    videos/

reports/
  sim2real/
    ONE_PAGER.md
    overview.md
  stage1/
    ONE_PAGER.md
    overview.md
    figures/
    images/
```
