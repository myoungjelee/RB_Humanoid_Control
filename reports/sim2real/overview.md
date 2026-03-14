# Sim-to-Real Overview (ROS2 Main Track)

## 1) 목적
이 문서는 `ros2_ws/` 기반 Sim-to-Real 메인 트랙의 진행 상태와 검증 결과를 기술적으로 정리한 문서입니다.  
Stage1(`reports/stage1/*`)은 baseline 자산으로 유지하고, 본 문서는 ROS2 메인 트랙이 **왜 이런 순서로 진행됐는지**, **각 단계에서 무엇을 구현했고 무엇으로 검증했는지**, **현재 어디까지 왔는지**를 기록합니다.

## 2) 현재 단계
- 상태: M0 완료, M1 완료, M2 완료, M3 완료, M4 완료, M5 standing hold 확보, M6 증빙 인프라 완료, M7 safety-on standing 완료, M8 대표 disturbance A/B 확보, M9 KPI 자동화 완료
- 현재 핵심 성과:
  - controller-only no-disturbance standing hold 확보
  - standing 실패의 핵심 원인을 `IMU frame interpretation mismatch`로 분리
  - observer 쪽 `imu_frame_mode=g1_imu_link` 보정으로 개선
  - safety-on 기준 `CONTROL_ACTIVE` 이후 60초 no-fall / no-safety-reason 확인
  - `113N x 0.10s` sagittal torso impulse 기준 `OFF 3/3 fall`, `ON 3/3 no-fall` 확인
- 다음 단계:
  - 포트폴리오 패키징

## 3) 왜 이 순서로 진행했는가
이 프로젝트는 처음부터 "잘 서는가?"를 바로 보는 대신, **실패 원인을 좁혀갈 수 있는 순서**로 구성했습니다.

### M0 먼저: 인터페이스를 고정
- 이유:
  - 토픽 이름, 조인트 순서, command mode가 중간에 바뀌면 이후 결과가 비교 불가능해지기 때문
- 잠근 항목:
  - `/clock` + `/rb/*`
  - `effort` command
  - joint ordering source
  - `control_rate_hz=200`, `sim.dt=0.005`

### M1 다음: 센서 경로 확인
- 이유:
  - controller 이전에 `/clock`, `/rb/joint_states`, `/rb/imu`가 정상인지 확인해야 하기 때문

### M2 다음: 제어 루프 자체 확인
- 이유:
  - fixed-rate controller가 돌아야 이후 command apply, standing, safety가 의미를 갖기 때문

### M3 다음: 명령이 실제로 적용되는지 확인
- 이유:
  - controller 계산값이 articulation에 실제 반영되는지 확인해야 "제어가 먹는다"를 말할 수 있기 때문

### M4 다음: safety를 분리 검증
- 이유:
  - 실무에서는 controller를 바로 actuator에 꽂지 않기 때문
  - `CLAMP / JOINT_LIMIT / TIMEOUT / TILT`를 개별로 분리 확인해야 이후 standing 중 safety 오작동도 해석 가능함

### M5 마지막: standing
- 이유:
  - 센서, 루프, 명령 적용, safety 구조를 확인한 뒤에야 standing 실패를 진짜 제어/관측 문제로 좁힐 수 있기 때문

## 4) M0 결정 항목(잠금 완료)
- Isaac Sim: `5.1`
- ROS2/OS: `Humble / Ubuntu 22.04`
- Command 인터페이스: `effort`
- Topic: `/clock` + `/rb/*`
- Joint ordering source: `ros2_ws/src/rb_bringup/config/joint_order_g1.yaml`
- Frame: `base_link`, `imu_link`
- Control timing: `control_rate_hz=200`, `sim.dt=0.005`, `substeps=1`, `decimation=1`
- Main backend path: `original G1 direct spawn + standalone World.step()`

## 5) 마일스톤별 구현/검증

### M1: 센서 파이프라인
- 구현:
  - `/clock`, `/rb/joint_states`, `/rb/imu` 브리지 경로 확인
- 왜 중요했나:
  - 센서 입력이 안 오면 controller와 standing은 전부 무의미하기 때문
- 검증:
  - topic publish 확인
  - `use_sim_time` 확인
- 증빙:
  - `reports/sim2real/images/standalone_backend/m1_standalone.png`

### M2: C++ controller skeleton
- 구현:
  - `rb_controller` 패키지 생성
  - 200Hz wall timer 기반 loop
  - `/rb/command_raw` 발행
  - dt/jitter 통계 출력
- 왜 중요했나:
  - 실기체 대응 구조에서 fixed-rate controller와 loop telemetry는 핵심이기 때문
- 검증:
  - `/rb/command_raw` publish
  - `dt_mean/dt_max/p95/miss_count`
- 증빙:
  - `reports/sim2real/images/standalone_backend/m2_controller_standalone.png`

### M3: command apply
- 구현:
  - OmniGraph subscribe -> articulation apply 경로 구성
- 왜 중요했나:
  - controller가 계산한 effort가 실제 로봇 모델에 적용되는지 확인해야 하기 때문
- 검증:
  - `/rb/command_raw` 발행
  - `joint_states` before/after 변화
- 증빙:
  - `reports/sim2real/images/standalone_backend/m3_command_standalone.png`

### M4: safety
- 구현:
  - `rb_safety` 레이어 정리
  - `CLAMP`, `JOINT_LIMIT`, `TIMEOUT`, `TILT`
  - 최근에는 `VELOCITY_LIMIT`, real command watchdog도 추가
- 왜 중요했나:
  - 실무 구조는 `controller -> safety -> actuator`이기 때문
- 검증:
  - 각 safety reason 개별 발동
- 증빙:
  - `reports/sim2real/images/standalone_backend/m4_clamp_standalone.png`
  - `reports/sim2real/images/standalone_backend/m4_joint_limit_standalone.png`
  - `reports/sim2real/images/standalone_backend/m4_timeout_standalone.png`
  - `reports/sim2real/images/standalone_backend/m4_tilt_standalone.png`
  - `reason=VELOCITY_LIMIT` terminal 증빙

### M5: standing

#### M5-1. baseline 확인: "아예 못 선다"를 먼저 확인
- 시작점:
  - no-disturbance에서도 `1~2초` 안쪽 전방 붕괴
- 왜 이 단계가 필요했나:
  - disturbance 이전에 stand 자체가 성립하는지부터 확인해야 하기 때문
- 구조 메모:
  - 현재 M5의 standing controller는 raw joint/IMU feedback를 직접 사용하는 경량 bring-up 구조다
  - state estimator 분리는 후속 milestone(M11)에서 진행한다
- 여기서 한 일:
  - `stand_pd_sanity` 경로 분리
  - controller-only bring-up
  - safety 영향과 stand 자체 문제를 분리

#### M5-2. pose / gain / actuator 튜닝
- 여기서 한 일:
  - `standing q_ref` 여러 버전 시도(`v4`, `v5`, `v6`)
  - `stand_effort_abs_max`, `tilt_kp_pitch`, `tilt_kd_pitch` 등 조정
  - ankle actuator authority 강화(`40/4/40 -> 60/6/40 -> 80/8/40`)
- 여기서 배운 점:
  - 자세와 발목 authority는 어느 정도 개선 효과가 있었지만,
  - 그것만으로는 핵심 failure mode를 바꾸지 못했음

#### M5-3. 관측성 강화
- 여기서 한 일:
  - `fall_event` 도입
  - `loop_stats`에 `ankle_pd_avg`, `ankle_cmd_avg`, `ankle_sat` 추가
  - `imu_raw_r/p`, `imu_bias_r/p` 추가
  - `sync marker(FIRST_SIM_STEP / CONTROL_ACTIVE)` 기반 로그 해석으로 전환
- 왜 중요했나:
  - "넘어졌다"는 사실보다 **왜 넘어졌는지**를 숫자로 분리할 수 있게 됨

#### M5-4. 핵심 원인 분리: IMU frame 해석 문제
- 관찰:
  - 실제로는 앞으로 넘어지는데 `tilt_p≈0`, `u_pitch≈0`가 반복됨
  - raw IMU를 보니 전방 붕괴 정보가 `pitch`가 아니라 `roll` 채널 변화로 들어오고 있었음
- 해석:
  - IMU가 고장난 것이 아니라,
  - controller가 `imu_link` raw orientation을 이미 body/control frame에 정렬된 값처럼 쓰고 있었음
- 결론:
  - 문제는 publisher보다 **observer frame interpretation**

#### M5-5. 해결: observer-side frame compensation
- 적용:
  - `tilt_axis_mode=swap_rp` 실험으로 먼저 확인
  - 이후 `imu_frame_mode=g1_imu_link`로 정식화
- 결과:
  - controller-only no-disturbance standing hold 확보
  - `fall_event.txt`에 `[NO_FALL_EVENT] capture_window_sec=60` 반복 확인
  - `60~90초` 이상 hold도 반복 관찰
- 현재 대표 증빙:
  - `logs/sim2real/_legacy/20260314-121949_m5_stand_sanity_qrefv7/m5/fall_event.txt`
  - `logs/sim2real/_legacy/20260314-121949_m5_stand_sanity_qrefv7/m5/sync_markers.txt`
  - `ros2_ws/src/rb_controller/config/scenarios/stand_pd_sanity.yaml`

### M6: evidence / artifact infrastructure
- 구현:
  - `FIRST_SIM_STEP`, `CONTROL_ACTIVE` sync marker
  - `fall_event.txt`, `sync_markers.txt`, `loop_post_sync.txt`, `loop_before_fall.txt`
  - tmuxp fixed-window capture
- 왜 중요했나:
  - standing이나 safety 결과를 나중에 다시 설명하려면 raw log와 요약 아티팩트가 함께 있어야 하기 때문
- 결과:
  - `logs/sim2real/m5/<run_id>/m5/*`, `logs/sim2real/m7/<run_id>/m7/*` 구조 정착

### M7: safety-on standing 재통합
- 구현:
  - `stand_pd_safecheck.yaml`
  - `m7_stand_safecheck.yaml`
  - `safety_enabled=true`, `effort_abs_max_default=18`, `tilt_limit=0.6rad`, `velocity_limit=8/12`
- 왜 중요했나:
  - controller-only로 서는 것과, safety까지 포함해 실무형 구조로 서는 것은 다르기 때문
- 검증:
  - `CONTROL_ACTIVE` 이후 60초 fixed window
  - `[NO_FALL_EVENT]`
  - `[NO_SAFETY_REASON]`
- 대표 증빙:
  - `logs/sim2real/_legacy/20260314-133954_m7_stand_safecheck/m7/fall_event.txt`
  - `logs/sim2real/_legacy/20260314-133954_m7_stand_safecheck/m7/reason_count.txt`
  - `logs/sim2real/_legacy/20260314-133954_m7_stand_safecheck/m7/sync_markers.txt`

### M8: disturbance A/B
- 구현:
  - `stand_pd_balance_base.yaml` 공통 baseline
  - runtime `enable_tilt_feedback` override로 `balance_off/on` 분리
  - `run_m8_pair.sh`로 같은 `RUN_ID` 아래 순차 실행
- 왜 중요했나:
  - standing이 된다는 것과, 같은 외란에서 balance feedback이 실제로 효과가 있다는 것은 다르기 때문
- 현재 대표 조건:
  - `113N x 0.10s` sagittal torso impulse
  - `CONTROL_ACTIVE + 3s` 기준 외란 시작
- 대표 결과:
  - `balance_off`: `3/3 fall`
  - `balance_on`: `3/3 no-fall`
- 대표 증빙:
  - `logs/sim2real/m8/20260314-184316/`
  - `logs/sim2real/m8/20260314-184442/`
  - `logs/sim2real/m8/20260314-184609/`

### M9: KPI/report automation
- 구현:
  - `extract_m8_kpi.py` 후처리 extractor 추가
  - `run_m8_pair.sh`, `run_m9_kpi.sh`, `run_m8_and_m9.sh`로 실행 레이어 분리
  - `M8 raw -> M9 summary` 구조 정리
- 왜 중요했나:
  - raw log만으로는 run-to-run 비교가 불편하고, 포트폴리오/면접에서 바로 읽을 수 있는 요약 문서가 필요했기 때문
- 결과:
  - `logs/sim2real/m9/<run_id>/balance_off_kpi.json`
  - `logs/sim2real/m9/<run_id>/balance_on_kpi.json`
  - `logs/sim2real/m9/<run_id>/comparison.json`
  - `logs/sim2real/m9/<run_id>/summary.md`
  - `logs/sim2real/m9/index.csv`
  가 자동 생성되도록 정리
- 대표 증빙:
  - `logs/sim2real/m9/20260314-184316/`

## 6) 현재 standing 해석
- 기존 문제:
  - controller가 `imu_link` raw orientation을 이미 body/control frame에 정렬된 값처럼 사용
- 결과:
  - 실제 전방 붕괴가 `pitch` 보정 루프로 안 들어감
  - gain을 아무리 조정해도 핵심 failure mode가 계속 반복됨
- 해결 방식:
  - publisher를 뒤집지 않음
  - observer에서 `imu_link -> control frame` 보정
- 현재 파라미터 이름:
  - `imu_frame_mode=g1_imu_link`

## 7) 현재 남은 작업
1. 포트폴리오 패키징 마감
2. `reason_count.txt` raw parsing 정리
3. 이후 robustness margin 확장

## 8) 아티팩트 경로
- 실행 로그: `logs/sim2real/<milestone>/<run_id>/`
- M5 대표 로그:
  - `logs/sim2real/m5/<run_id>/m5/fall_event.txt`
  - `logs/sim2real/m5/<run_id>/m5/sync_markers.txt`
  - `logs/sim2real/m5/<run_id>/m5/loop_post_sync.txt`
  - `logs/sim2real/m5/<run_id>/m5/loop_before_fall.txt`
- 요약 문서:
  - `README.md`
  - `reports/sim2real/overview.md`
  - `reports/sim2real/ONE_PAGER.md`
  - `STATUS.md`
- M9 요약 경로:
  - `logs/sim2real/m9/<run_id>/balance_off_kpi.json`
  - `logs/sim2real/m9/<run_id>/balance_on_kpi.json`
  - `logs/sim2real/m9/<run_id>/comparison.json`
  - `logs/sim2real/m9/<run_id>/summary.md`
