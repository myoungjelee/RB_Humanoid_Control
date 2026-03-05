# Sim-to-Real Overview (ROS2 Main Track)

## 1) 목적
이 문서는 `ros2_ws/` 기반 Sim-to-Real 트랙의 진행 상태와 검증 결과를 요약한다.
Stage1(`reports/stage1/*`)은 baseline 증거로 유지하고, 본 문서는 메인트랙 결과를 기록한다.

## 2) 현재 단계
- 상태: M0 완료, M1 완료, M2 완료, M3 완료, M4 진행 준비
- 기준 문서:
  - `AGENTS.md`
  - `MASTER_PLAN.md`
  - `STATUS.md`

## 3) M0 결정 항목(잠금 완료)
- Isaac Sim: `5.1`
- ROS2/OS: `Humble / Ubuntu 22.04`
- RT kernel: `No` (일반 커널, RT-ready 계측만)
- Command 인터페이스: `effort`
- Topic: `/clock` + `/rb/*` (`/rb/joint_states`, `/rb/imu`, `/rb/command*`, `/rb/state`)
- Joint ordering source: `ros2_ws/src/rb_bringup/config/joint_order_g1.yaml`
- Frame: `base_link`, `imu_link`
- Control timing: `control_rate_hz=200`, `sim.dt=0.005`, `substeps=1`, `decimation=1`

## 4) 완료 항목(M1)
- `/clock` publish 확인
- `/rb/joint_states` publish 확인
- `/rb/imu` publish 확인
- `use_sim_time` 동작 확인
- 증빙 이미지: `reports/sim2real/images/m1.png`
- 저장 stage: `sim/isaac_scenes/g1_stage.usd`

## 5) 완료 항목(M2)
- `rb_controller` C++ 패키지 생성/빌드 완료
- 0 torque publish(`/rb/command_raw`) 확인
- dt/jitter 통계 출력(`dt_mean/dt_max/p95/miss_count`) 확인
- 증빙 이미지: `reports/sim2real/images/m2_controller.png`

## 6) 완료 항목(M3)
- OmniGraph command apply 경로 구성:
  - `ROS2SubscribeJointState` -> `IsaacArticulationController`
  - topic: `/rb/command_raw`
- `command_apply=True` 그래프 구성 로그 확인
- `/rb/command_raw` 200Hz 근처 발행 확인
- `/rb/joint_states` before/after diff 변화 확인
- 증빙 이미지: `reports/sim2real/images/m3_command.png`
- 보조 로그: `logs/sim2real/m3/*`

## 7) 다음 검증 항목(M4)
- safety gate(clamp/timeout/tilt) 추가
- 안전 개입(reason) 로그/토픽 증빙

## 8) 아티팩트 경로
- 실행 로그: `logs/sim2real/<timestamp>/`
- 요약 리포트: `reports/sim2real/overview.md`
- one-pager: `reports/sim2real/ONE_PAGER.md`
