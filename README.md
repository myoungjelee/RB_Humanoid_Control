# RB_Humanoid_Control

## 프로젝트 요약
Isaac Sim 5.1 + ROS2 Humble 기반으로, 휴머노이드 제어 경로를  
`센서 -> 추정/관측 -> 제어(C++) -> 안전 -> 로그/KPI` 구조로 설계하고 검증하는 Sim-to-Real 포트폴리오 프로젝트입니다.

핵심 목표는 "시뮬에서만 잠깐 도는 데모"가 아니라,  
**실기체 백엔드로 바꿔도 유지될 인터페이스와 검증 습관을 갖춘 제어 스택**을 만드는 것입니다.

## 현재 핵심 성과
- ROS2 메인 트랙 기준 M1~M4 재검증 완료
  - safety reason: `CLAMP`, `JOINT_LIMIT`, `TIMEOUT`, `TILT`, `VELOCITY_LIMIT`
- M5 no-disturbance standing에서 **controller-only 장시간 hold** 확보
- M7 safety-on standing에서 **CONTROL_ACTIVE 기준 60초 no-fall / no-safety-reason** 확인
- standing 실패의 핵심 원인 분리 완료
  - 문제: gain 부족보다 **`imu_link` 축을 control/body frame처럼 직접 읽던 해석 불일치**
  - 해결: publisher 수정이 아니라 observer 쪽 **`imu_frame_mode=g1_imu_link` 보정**
- 다음 단계
  - 같은 torso push에서 balance feedback OFF/ON 비교
  - KPI 자동화

## 내가 만든 것
- `/clock`, `/rb/joint_states`, `/rb/imu`, `/rb/command*` 기준 ROS2 인터페이스 고정
- `rb_controller` C++ 200Hz loop + dt/jitter 관측
- `rb_safety` watchdog / tilt / clamp / velocity-limit 구조
- tmuxp + sync marker 기반 실험 오케스트레이션
- Stage1 baseline과 Sim2Real main track 분리 운영

## 시스템 구조
```text
Isaac Sim (ROS2 Bridge)
  -> /clock, /rb/joint_states, /rb/imu
  -> rb_state_estimator / observer
  -> rb_controller (C++/rclcpp, 200Hz, dt/jitter telemetry)
  -> rb_safety
  -> /rb/command_safe
  -> logs / KPI / reports
```

## 고정한 엔지니어링 결정
- Isaac Sim 5.1 / ROS2 Humble / Ubuntu 22.04
- Command mode: `effort`
- Namespace: `/clock` + `/rb/*`
- Timing: `control_rate_hz=200`, `sim.dt=0.005`, `substeps=1`, `decimation=1`
- Joint ordering source: `ros2_ws/src/rb_bringup/config/joint_order_g1.yaml`
- Main path: `original G1 direct spawn + standalone World.step()`

## 마일스톤을 이 순서로 진행한 이유
제어 프로젝트는 바로 "잘 서는지"부터 보면 어디가 잘못됐는지 분리하기 어렵습니다.  
그래서 이 프로젝트는 **센서 -> 제어 루프 -> 명령 적용 -> 안전 -> standing** 순서로 쌓았습니다.

- **M0 결정 잠금**
  - 왜 먼저 했나: 토픽, 조인트 순서, command mode가 바뀌면 이후 결과가 전부 흔들리기 때문
- **M1 센서 경로 확인**
  - 왜 다음인가: `/clock`, `/rb/joint_states`, `/rb/imu`가 안 오면 나머지는 전부 무의미하기 때문
- **M2 제어 루프 확인**
  - 왜 필요한가: fixed-rate controller와 dt/jitter 관측이 있어야 "제어가 돌았다"를 증명할 수 있기 때문
- **M3 명령 적용 확인**
  - 왜 필요한가: controller가 계산한 값이 실제 articulation에 적용되는지 분리 확인해야 하기 때문
- **M4 safety 분리 검증**
  - 왜 필요한가: controller를 맹신하지 않고 마지막에 safety layer가 개입하는 구조를 증명해야 하기 때문
- **M5 standing**
  - 왜 마지막인가: 센서/루프/적용/안전이 확인된 뒤에야 "왜 못 서는지"를 진짜 제어 문제로 좁힐 수 있기 때문

## 마일스톤별 구현과 검증
| 단계 | 무엇을 했는가 | 무엇으로 검증했는가 | 증빙 |
| --- | --- | --- | --- |
| M1 | `/clock`, `/rb/joint_states`, `/rb/imu` 브리지 확인 | topic publish 확인 | [M1 이미지](reports/sim2real/images/standalone_backend/m1_standalone.png) |
| M2 | `rb_controller` 200Hz loop, dt/jitter 출력 | `/rb/command_raw`, `dt_mean/p95/max` | [M2 이미지](reports/sim2real/images/standalone_backend/m2_controller_standalone.png) |
| M3 | command apply 경로 연결 | `joint_states` before/after 변화 | [M3 이미지](reports/sim2real/images/standalone_backend/m3_command_standalone.png) |
| M4 | safety gating 구조 정리 | `CLAMP`, `JOINT_LIMIT`, `TIMEOUT`, `TILT`, `VELOCITY_LIMIT` 개별 증빙 | [CLAMP](reports/sim2real/images/standalone_backend/m4_clamp_standalone.png), [JOINT_LIMIT](reports/sim2real/images/standalone_backend/m4_joint_limit_standalone.png), [TIMEOUT](reports/sim2real/images/standalone_backend/m4_timeout_standalone.png), [TILT](reports/sim2real/images/standalone_backend/m4_tilt_standalone.png) |
| M5 | standing hold 확보, 실패 원인 분리 | `fall_event`, `sync_markers`, `loop_stats`, GUI 관찰 | [STATUS](STATUS.md), [stand scenario](ros2_ws/src/rb_controller/config/scenarios/stand_pd_sanity.yaml) |
| M7 | safety-on standing 재통합 | `NO_FALL_EVENT`, `NO_SAFETY_REASON`, `CONTROL_ACTIVE` 기준 60초 hold | [safecheck scenario](ros2_ws/src/rb_controller/config/scenarios/stand_pd_safecheck.yaml), [safecheck tmuxp](ops/tmuxp/m7_stand_safecheck.yaml) |

## M5에서 실제로 해결한 문제
초기 M5 standing은 아무리 gain을 만져도 `1~2초` 안쪽에서 계속 전방 붕괴했습니다.  
raw IMU/bias/tilt observability를 추가해서 보니, 실제 forward fall 정보가 controller `pitch` 채널이 아니라 `roll` 쪽으로 들어오고 있었습니다.

즉 IMU가 고장난 게 아니라, **`imu_link` 기준 축을 controller가 기대한 body/control frame 축처럼 곧바로 읽고 있던 것**이 핵심 문제였습니다.  
이후 observer에 `imu_frame_mode=g1_imu_link` 보정을 넣자 controller-only no-disturbance standing hold가 장시간 유지되기 시작했습니다.

## 증빙 링크
### 핵심 문서
- Sim2Real overview: [reports/sim2real/overview.md](reports/sim2real/overview.md)
- Sim2Real one-pager: [reports/sim2real/ONE_PAGER.md](reports/sim2real/ONE_PAGER.md)
- 현재 실험 기록: [STATUS.md](STATUS.md)
- 영상 캡처 계획: [VIDEO_CAPTURE_GUIDE.md](VIDEO_CAPTURE_GUIDE.md)
- 마스터 플랜: [MASTER_PLAN.md](MASTER_PLAN.md)

### Stage1 / 비교 기준
- Stage1 overview: [reports/stage1/overview.md](reports/stage1/overview.md)
- Stage1 one-pager: [reports/stage1/ONE_PAGER.md](reports/stage1/ONE_PAGER.md)

### 현재 M5 로그 아티팩트
- 최신 standing scenario: [stand_pd_sanity.yaml](ros2_ws/src/rb_controller/config/scenarios/stand_pd_sanity.yaml)
- run별 대표 아티팩트 위치
  - `logs/sim2real/m5/<run_id>/m5/fall_event.txt`
  - `logs/sim2real/m5/<run_id>/m5/sync_markers.txt`
  - `logs/sim2real/m5/<run_id>/m5/loop_post_sync.txt`
  - `logs/sim2real/m5/<run_id>/m5/loop_before_fall.txt`

### 현재 M7 로그 아티팩트
- safety-on standing scenario: [stand_pd_safecheck.yaml](ros2_ws/src/rb_controller/config/scenarios/stand_pd_safecheck.yaml)
- run별 대표 아티팩트 위치
  - `logs/sim2real/m7/<run_id>/m7/fall_event.txt`
  - `logs/sim2real/m7/<run_id>/m7/reason_count.txt`
  - `logs/sim2real/m7/<run_id>/m7/sync_markers.txt`
  - `logs/sim2real/m7/<run_id>/m7/loop_post_sync.txt`
  - `logs/sim2real/m7/<run_id>/m7/loop_before_fall.txt`

## 현재 진행 상태 (2026-03-14)
- M0 Decision Lock: 완료
- M1 Sensor Pipeline: 완료
- M2 C++ Controller Skeleton: 완료
- M3 Command Apply: 완료
- M4 Safety Layer: 완료
- M5 Controller-only standing hold: 완료
- M6 Evidence / Artifact Infrastructure: 완료
- M7 Safety-on standing: 완료
- 다음 게이트: disturbance A/B

## 다음 단계
- M8: disturbance A/B 및 recovery evidence
- M9: KPI/report 자동화
- M10: 포트폴리오 패키징 마감
