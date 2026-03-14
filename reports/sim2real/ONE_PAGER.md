# Sim-to-Real ONE_PAGER (ROS2 Main Track)

## 한 줄 요약
Isaac Sim과 실기체에서 재사용 가능한 ROS2 제어 스택 인터페이스를 고정하고,  
센서 -> 제어 -> 안전 -> standing -> safety 재통합까지 단계별로 검증한 휴머노이드 Sim-to-Real 포트폴리오입니다.

## 무엇을 만들었는가
- `/clock`, `/rb/joint_states`, `/rb/imu`, `/rb/command*` 기준 ROS2 인터페이스
- `rb_controller` C++ 200Hz control loop + dt/jitter 관측
- `rb_safety` safety gating
- sync marker 기반 로그/실험 오케스트레이션
- M8 raw -> M9 summary 자동 요약 경로
- standing 디버깅용 observer/debug 모듈화
- safety-on standing 검증용 시나리오와 증빙 아티팩트 경로

## 왜 이 순서로 진행했는가
- M0: 인터페이스를 먼저 고정해야 이후 실험이 비교 가능해짐
- M1: 센서가 안 오면 제어는 무의미
- M2: fixed-rate controller가 먼저 살아야 함
- M3: 명령이 articulation에 실제 적용되는지 확인해야 함
- M4: controller를 바로 actuator에 연결하지 않고 safety layer를 검증해야 함
- M5: 그다음에야 standing 실패를 진짜 제어/관측 문제로 좁힐 수 있음
- M6: standing 결과를 재현 가능한 로그 아티팩트로 남겨야 이후 설명 가능함
- M7: controller-only로 서는 것과 safety를 포함해 서는 것은 다르기 때문에 재통합 검증이 필요함

## 마일스톤별 검증
| 단계 | 구현 | 검증 | 증빙 |
| --- | --- | --- | --- |
| M1 | `/clock`, `/rb/joint_states`, `/rb/imu` | topic publish | [m1_standalone.png](images/standalone_backend/m1_standalone.png) |
| M2 | `rb_controller` 200Hz loop | `/rb/command_raw`, dt/jitter | [m2_controller_standalone.png](images/standalone_backend/m2_controller_standalone.png) |
| M3 | command apply graph | joint state 변화 | [m3_command_standalone.png](images/standalone_backend/m3_command_standalone.png) |
| M4 | safety gating | `CLAMP`, `JOINT_LIMIT`, `TIMEOUT`, `TILT`, `VELOCITY_LIMIT` | [m4_clamp](images/standalone_backend/m4_clamp_standalone.png), [m4_joint_limit](images/standalone_backend/m4_joint_limit_standalone.png), [m4_timeout](images/standalone_backend/m4_timeout_standalone.png), [m4_tilt](images/standalone_backend/m4_tilt_standalone.png), `reason=VELOCITY_LIMIT` terminal 캡처 |
| M5 | controller-only standing hold | `fall_event`, `sync marker`, `loop_stats`, GUI 관찰 | [STATUS](../../STATUS.md), [stand_pd_sanity.yaml](../../ros2_ws/src/rb_controller/config/scenarios/stand_pd_sanity.yaml) |
| M6 | 증빙/아티팩트 인프라 | `fall_event.txt`, `sync_markers.txt`, `loop_post_sync.txt`, `loop_before_fall.txt` | [overview.md](overview.md), [STATUS](../../STATUS.md) |
| M7 | safety-on standing 재통합 | `CONTROL_ACTIVE` 기준 60초 `NO_FALL_EVENT`, `NO_SAFETY_REASON` | [stand_pd_safecheck.yaml](../../ros2_ws/src/rb_controller/config/scenarios/stand_pd_safecheck.yaml), [m7_stand_safecheck.yaml](../../ops/tmuxp/m7_stand_safecheck.yaml) |
| M8 | disturbance A/B | `113N x 0.10s`에서 `OFF 3/3 fall`, `ON 3/3 no-fall` | [stand_pd_balance_base.yaml](../../ros2_ws/src/rb_controller/config/scenarios/stand_pd_balance_base.yaml), [run_m8_pair.sh](../../ops/run_m8_pair.sh) |
| M9 | KPI/report 자동화 | `comparison.json`, `summary.md`, `m9/index.csv` 자동 생성 | [extract_m8_kpi.py](../../scripts/sim2real/extract_m8_kpi.py), [run_m9_kpi.sh](../../ops/run_m9_kpi.sh) |

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
  - 대표 로그: `logs/sim2real/_legacy/20260314-121949_m5_stand_sanity_qrefv7/m5/fall_event.txt`

## M7에서 확인한 것
- controller-only로 서는 세팅을 그대로 가져가고 safety만 다시 켰음
- `CONTROL_ACTIVE` 이후 60초 고정 관측창 동안
  - `[NO_FALL_EVENT]`
  - `[NO_SAFETY_REASON]`
  를 확인함
- 의미:
  - safety가 불필요하게 개입하지 않으면서
  - 현재 standing baseline을 유지할 수 있음을 확인

## 현재 상태
- M0 완료
- M1 완료
- M2 완료
- M3 완료
- M4 완료
- M5 controller-only standing hold 확보
- M6 증빙/아티팩트 인프라 완료
- M7 safety-on standing 완료
- M8 대표 disturbance A/B 확보
- M9 KPI/report 자동화 완료
- 다음 단계: 포트폴리오 패키징

## 핵심 링크
- 랜딩 페이지: [README.md](../../README.md)
- 기술 복기: [overview.md](overview.md)
- 현재 실험 기록: [STATUS.md](../../STATUS.md)
