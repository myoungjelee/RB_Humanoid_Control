# Sim-to-Real ONE_PAGER (ROS2 Main Track)

## 한 줄 요약
Isaac Sim과 실기체에서 재사용 가능한 ROS2 제어 스택(추정/제어/안전/로그) 인터페이스를 고정하고, 단계별 검증 증거를 축적한다.

## 시스템 블록
`Sensors(/rb/joint_states,/rb/imu,/clock)` -> `Estimator(/rb/state)` -> `Controller(/rb/command*)` -> `Safety(/rb/safety/state)` -> `KPI Logger`

## 트랙 정책
- Main: `ros2_ws/` Sim-to-Real
- Baseline: `scripts/stage1/` 유지(비교 기준선)

## 검증 기준
- 토픽 증빙: `ros2 topic hz/echo`
- 로그 증빙: `logs/sim2real/<timestamp>/run.log`
- 리포트 증빙: KPI 표/그래프 + markdown summary

## 현재 상태
- M1 완료: `/clock`, `/rb/joint_states`, `/rb/imu` publish 확인
- M1 증빙: `reports/sim2real/images/m1.png`
- M2 완료: `rb_controller` C++ 200Hz loop + dt/jitter + `/rb/command_raw` 검증
- M2 증빙: `reports/sim2real/images/m2_controller.png`
- 저장 stage: `sim/isaac_scenes/g1_stage.usd`

## M1 증빙 이미지
![M1 Topic Proof](images/m1.png)

## M2 증빙 이미지
![M2 Controller Proof](images/m2_controller.png)

## 다음 TODO
- M3 command apply(`rb/command` -> Isaac articulation) 검증
