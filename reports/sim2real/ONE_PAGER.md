# Sim-to-Real ONE_PAGER (ROS2 Main Track)

## 한 줄 요약
Isaac Sim과 실기체에서 재사용 가능한 ROS2 제어 스택(추정/제어/안전/로그) 인터페이스를 고정하고, 단계별 검증 증거를 축적한다.

## 시스템 블록
`Sensors(/joint_states,/imu,/clock)` -> `Estimator(/rb/state)` -> `Controller(/command/*)` -> `Safety` -> `KPI Logger`

## 트랙 정책
- Main: `ros2_ws/` Sim-to-Real
- Baseline: `scripts/stage1/` 유지(비교 기준선)

## 검증 기준
- 토픽 증빙: `ros2 topic hz/echo`
- 로그 증빙: `logs/ros2_runs/<timestamp>/run.log`
- 리포트 증빙: KPI 표/그래프 + markdown summary

## 현재 TODO
- M0 인터페이스 결정 확정
- M1 센서 토픽 파이프라인 확보
- M2 controller skeleton loop 연결
