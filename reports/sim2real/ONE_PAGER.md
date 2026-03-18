# Sim-to-Real ONE_PAGER (ROS2 Main Track)

## 한 줄 요약
Isaac Sim과 실기체에서 재사용 가능한 ROS2 제어 스택 인터페이스를 고정하고,  
센서 -> 제어 -> 안전 -> standing -> disturbance -> KPI 자동화까지 단계별로 검증한 휴머노이드 Sim-to-Real 포트폴리오입니다.

## 무엇을 만들었는가
- `/clock`, `/rb/joint_states`, `/rb/imu`, `/rb/command*` 기준 ROS2 인터페이스
- `rb_controller` C++ 200Hz control loop + dt/jitter 관측
- `rb_safety` safety gating
- sync marker 기반 로그/실험 오케스트레이션
- M8 raw -> M9 summary 자동 요약 경로
- standing 디버깅용 observer/debug 모듈화

## 대표 결과
| 단계 | 구현 | 검증 | 증빙 |
| --- | --- | --- | --- |
| M1~M4 | 센서, controller loop, command apply, safety pipeline | topic publish / dt-jitter / reason별 safety 발동 | [m1](images/standalone_backend/m1_standalone.png), [m2](images/standalone_backend/m2_controller_standalone.png), [m3](images/standalone_backend/m3_command_standalone.png), [m4 velocity](images/standalone_backend/m4_velocity_limit_standalone.png) |
| M5 | controller-only standing hold | `fall_event`, `sync marker`, `loop_stats`, GUI 관찰 | [overview.md](overview.md), [stand_pd_sanity.yaml](../../ros2_ws/src/rb_controller/config/scenarios/stand_pd_sanity.yaml) |
| M7 | safety-on standing 재통합 | `CONTROL_ACTIVE` 기준 60초 `NO_FALL_EVENT`, `NO_SAFETY_REASON` | [m7_t0.png](images/standalone_backend/m7_t0.png), [m7_t60.png](images/standalone_backend/m7_t60.png), [stand_pd_safecheck.yaml](../../ros2_ws/src/rb_controller/config/scenarios/stand_pd_safecheck.yaml) |
| M8 | disturbance A/B | `113N x 0.10s`에서 `OFF 3/3 fall`, `ON 3/3 no-fall` | [m8_disturb_tilted.png](images/standalone_backend/m8_disturb_tilted.png), [m8_disturb_recovered.png](images/standalone_backend/m8_disturb_recovered.png), [run_m8_pair.sh](../../ops/run_m8_pair.sh) |
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

## 대표 자산
- 최종 시연영상: [GitHub inline demo](https://github.com/user-attachments/assets/4e70156b-aca6-4c3a-859f-7526fa2f511e)
- 시스템 구조도: [system_architecture.png](images/standalone_backend/system_architecture.png)
- 검증 범위 / 다음 단계: [milestone_roadview.png](images/standalone_backend/milestone_roadview.png)

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
- 다음 단계: M10 observer / estimator refinement

## 핵심 링크
- 랜딩 페이지: [README.md](../../README.md)
- 기술 복기: [overview.md](overview.md)
- 현재 실험 기록: [STATUS.md](../../STATUS.md)
- 전체 로드맵: [MASTER_PLAN.md](../../MASTER_PLAN.md)
