# Sim-to-Real Overview (ROS2 Main Track)

## 1) 목적
이 문서는 `ros2_ws/` 기반 Sim-to-Real 트랙의 진행 상태와 검증 결과를 요약한다.
Stage1(`reports/stage1/*`)은 baseline 증거로 유지하고, 본 문서는 메인트랙 결과를 기록한다.

## 2) 현재 단계
- 상태: M0/M1 준비 단계
- 기준 문서:
  - `AGENTS.md`
  - `MASTER_PLAN.md`
  - `STATUS.md`

## 3) M0 결정 항목(고정 필요)
- ROS2 distro / OS
- Command 인터페이스(`effort` 또는 `position`)
- Joint ordering(`/joint_states.name[]`)
- Topic/frame 네이밍
- Control rate / physics dt / decimation

## 4) 다음 검증 항목(M1)
- `/clock` publish 확인
- `/joint_states` publish 확인
- `/imu` publish 확인
- `use_sim_time` 동작 확인

## 5) 아티팩트 경로
- 실행 로그: `logs/ros2_runs/<timestamp>/`
- 요약 리포트: `reports/sim2real/overview.md`
- one-pager: `reports/sim2real/ONE_PAGER.md`
