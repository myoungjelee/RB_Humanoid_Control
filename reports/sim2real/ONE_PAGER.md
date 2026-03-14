# Sim-to-Real ONE_PAGER (ROS2 Main Track)

## 한 줄 요약
Isaac Sim과 실기체에서 재사용 가능한 ROS2 제어 스택 인터페이스를 고정하고,  
센서 -> 제어 -> 안전 -> standing까지 단계별로 검증한 휴머노이드 Sim-to-Real 포트폴리오입니다.

## 무엇을 만들었는가
- `/clock`, `/rb/joint_states`, `/rb/imu`, `/rb/command*` 기준 ROS2 인터페이스
- `rb_controller` C++ 200Hz control loop + dt/jitter 관측
- `rb_safety` safety gating
- sync marker 기반 로그/실험 오케스트레이션
- standing 디버깅용 observer/debug 모듈화

## 왜 이 순서로 진행했는가
- M0: 인터페이스를 먼저 고정해야 이후 실험이 비교 가능해짐
- M1: 센서가 안 오면 제어는 무의미
- M2: fixed-rate controller가 먼저 살아야 함
- M3: 명령이 articulation에 실제 적용되는지 확인해야 함
- M4: controller를 바로 actuator에 연결하지 않고 safety layer를 검증해야 함
- M5: 그다음에야 standing 실패를 진짜 제어/관측 문제로 좁힐 수 있음

## 마일스톤별 검증
| 단계 | 구현 | 검증 | 증빙 |
| --- | --- | --- | --- |
| M1 | `/clock`, `/rb/joint_states`, `/rb/imu` | topic publish | [m1_standalone.png](images/standalone_backend/m1_standalone.png) |
| M2 | `rb_controller` 200Hz loop | `/rb/command_raw`, dt/jitter | [m2_controller_standalone.png](images/standalone_backend/m2_controller_standalone.png) |
| M3 | command apply graph | joint state 변화 | [m3_command_standalone.png](images/standalone_backend/m3_command_standalone.png) |
| M4 | safety gating | clamp/joint_limit/timeout/tilt | [m4_clamp](images/standalone_backend/m4_clamp_standalone.png), [m4_joint_limit](images/standalone_backend/m4_joint_limit_standalone.png), [m4_timeout](images/standalone_backend/m4_timeout_standalone.png), [m4_tilt](images/standalone_backend/m4_tilt_standalone.png) |
| M5 | standing hold | fall_event, sync marker, loop stats | [STATUS](../../STATUS.md) |

## M5에서 실제로 해결한 것
- 초기에는 no-disturbance standing도 `1~2초` 안쪽 전방 붕괴
- gain/pose/actuator만으로는 결정적 개선이 안 됨
- raw IMU/bias/tilt 로그를 추가해 확인한 결과,
  - 실제 전방 붕괴 정보가 `pitch`가 아니라 `roll` 채널로 들어오고 있었음
- 원인:
  - IMU 고장 아님
  - `imu_link` 축을 controller body/control frame처럼 직접 읽고 있었음
- 해결:
  - observer에서 `imu_link -> control frame` 보정
  - 현재 파라미터: `imu_frame_mode=g1_imu_link`
- 결과:
  - controller-only no-disturbance standing hold 확보
  - 대표 로그: `logs/sim2real/20260314-121949_m5_stand_sanity_qrefv7/m5/fall_event.txt`

## 현재 상태
- M0 완료
- M1 완료
- M2 완료
- M3 완료
- M4 완료
- M5 standing hold 확보
- 다음 단계: safety 재통합 -> disturbance A/B -> KPI 정리

## 핵심 링크
- 랜딩 페이지: [README.md](../../README.md)
- 기술 복기: [overview.md](overview.md)
- 현재 실험 기록: [STATUS.md](../../STATUS.md)
