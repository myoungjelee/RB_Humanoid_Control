# MASTER PLAN — RB_Humanoid_Control

## 1. 문서 목적

이 문서는 `RB_Humanoid_Control` 프로젝트의 전체 시스템 로드맵이다.

목적은 두 가지다.

1. 현재까지 시스템을 어떤 순서로 구축했는지 설명한다.
2. 이후 프로젝트를 어떤 방향으로 확장할지 정의한다.

즉 이 문서는 단순 TODO 목록이 아니라,
휴머노이드 제어 시스템을 단계적으로 구축하고 검증한 과정을 설명하는
엔지니어링 로드맵 문서다.

---

# 2. 프로젝트 최종 목표

이 프로젝트의 최종 목표는 다음을 엔지니어링 증거로 보여주는 것이다.

- ROS2 기반 휴머노이드 제어 스택을 직접 구축했다.
- 센서 / 관측 / 제어 / 안전 / 기록이 분리되는 구조를 설계했다.
- Sim 데모가 아니라 실기체에 연결 가능한 인터페이스 구조를 갖췄다.
- standing / disturbance / safety / KPI까지 하나의 실험 흐름으로 검증했다.

즉 이 프로젝트는

"알고리즘 설명 프로젝트"가 아니라

"로봇 제어 시스템 엔지니어링 프로젝트"

이다.

---

# 3. 메인 트랙 정의

## 3.1 메인 트랙

메인 트랙은

ROS2 기반 Sim-to-Real control stack

이다.

현재 구현의 중심은 다음과 같다.

- `ros2_ws/` 기반 controller / safety / bringup
- Isaac Sim + original G1 asset + direct spawn
- standalone `World.step()`

최종적으로는 아래 책임 분리를 목표로 한다.

- Estimator / Observer
- Planner / Behavior
- Controller
- Safety
- Logger / Recorder

즉 지금은 하나의 스택 안에서 검증을 진행하고,
최종적으로는 책임이 분리된 ROS2 시스템으로 정리한다.

---

## 3.2 baseline 트랙

기존 Stage1 / Gym / IsaacLab 실험은

baseline / archive / reference

로 유지한다.

즉

Stage1 = 참고 자산
ROS2 stack = 메인 시스템

---

# 4. 기술 방향 고정

다음 기술 선택은 프로젝트에서 고정한다.

Robot model
G1

Command mode
effort

Topic namespace
/clock
/rb/\*

Joint ordering source
`ros2_ws/src/rb_bringup/config/joint_order_g1.yaml`

Simulation backend
original G1 asset
direct spawn
standalone `World.step()`

Entry point
`main.py`

---

# 5. 전체 마일스톤

프로젝트는 다음 milestone 순서로 진행한다.

- `M0 → M6`: 시스템 bring-up 단계
- `M7 → M9`: 현재 마감선 단계
- `M10 이후`: 연구/확장 단계

---

# M0 — Interface Lock

목표

토픽 / 프레임 / 조인트 순서 / command mode를 고정한다.

왜 중요한가

이 단계가 흔들리면 이후 실험 결과가 비교 불가능해진다.

결과

- `/clock`
- `/rb/joint_states`
- `/rb/imu`
- `/rb/command_raw`

interface 확정

---

# M1 — Sensor Pipeline

목표

센서 입력 경로를 검증한다.

확인

- `/clock`
- `/rb/joint_states`
- `/rb/imu`

핵심 질문

센서 데이터가 controller까지 정상적으로 전달되는가

---

# M2 — Controller Loop / dt-jitter

목표

controller control loop timing을 확보한다.

조건

- 200Hz loop
- dt statistics logging

확인

- `dt_mean`
- `dt_max`
- `dt_p95`
- tick miss

핵심 질문

제어 루프가 고정 주기로 도는가

---

# M3 — Command Apply

목표

controller output이 실제 articulation에 적용되는지 확인한다.

경로

controller
→ `/rb/command_raw`
→ articulation apply

핵심 질문

controller 계산값이 실제 로봇 모델에 반영되는가

---

# M4 — Safety Layer

목표

controller와 actuator 사이에 safety layer를 구축한다.

구조

controller
→ command_raw
→ safety
→ command_safe
→ actuator

safety 항목

- `CLAMP`
- `JOINT_LIMIT`
- `TIMEOUT`
- `TILT`
- `VELOCITY_LIMIT`

핵심 질문

controller를 바로 actuator에 연결하지 않고
안전하게 한 번 더 검증할 수 있는가

---

# M5 — Stand Stabilization

목표

no-disturbance standing을 확보한다.

현재 M5의 standing baseline은 이후 `M7 ~ M9`까지 공통으로 재사용하는
standing controller 기준점이다.

active 실행 경로는 현재

`rb_estimation -> rb_standing_controller -> rb_hardware_interface -> rb_safety`

구조를 기준으로 정리되어 있으며,
후속 milestone에서는 이 경로를 유지한 채 calibration / metrics / control 확장을 진행한다.

핵심 작업

- pose tuning
- gain tuning
- tilt feedback tuning
- IMU frame validation
- actuator authority 확인

핵심 질문

왜 로봇이 서지 못하는가

- pose 문제인가
- gain 문제인가
- frame 문제인가
- observer 문제인가

핵심 발견

기존 controller는 `imu_link` raw orientation을
이미 body/control frame에 정렬된 roll/pitch처럼 직접 사용하고 있었다.

그 결과 실제 전방 붕괴가 `pitch`가 아니라 `roll` 채널 변화로 들어갔다.

해결

IMU publisher를 뒤집지 않고,
observer에서 `imu_link -> control frame` 보정을 적용한다.

현재 기준 구현은 `imu_frame_mode=g1_imu_link`다.

현재 상태

controller-only no-disturbance standing 확보

---

# M6 — Evidence / Run Artifact Infrastructure

목표

실험 결과를 재현 가능한 아티팩트 형태로 남기는 구조를 구축한다.

자동 기록

- run log
- params dump
- sync markers
- fall event
- 초기/후반 loop window
- summary skeleton

출력 구조

- `logs/sim2real/<milestone>/<run_id>/`
- `reports/sim2real/`
- `artifacts/`

핵심 질문

한 번의 실험을 나중에 다시 보고 설명할 수 있는가

---

# 6. 현재 마감선 Milestone

재지원 전 반드시 완료할 범위

---

# M7 — Safety-On Standing

목표

`safety_enabled=true` 상태에서도 standing을 유지한다.

확인

- `TILT`
- `TIMEOUT`
- `CLAMP`
- `VELOCITY_LIMIT`

기대 결과

safety가 정상 동작하면서
standing을 불필요하게 끊지 않는다.

---

# M8 — Disturbance A/B Test

목표

같은 외란에서 balance feedback 효과를 증명한다.

구조

- balance feedback OFF
- balance feedback ON

같은 disturbance

예

- torso impulse (1차)
- lateral push (후속)

목표 결과

- OFF → 바로 넘어짐
- ON → 더 오래 버팀

또는

- 둘 다 survive하지만
- ON의 peak tilt가 더 작음

현재 대표 증빙(2026-03-14 기준)

- `113N x 0.10s` sagittal torso impulse
- `balance feedback OFF`: `3/3 fall`
- `balance feedback ON`: `3/3 no-fall`

설명

이 milestone의 OFF/ON은 외란 유무가 아니라
같은 torso push에 대해 balance feedback을 끈 경우와 켠 경우를 비교한다.

---

# M9 — KPI Automation

목표

run 결과를 자동 요약하고 run 간 비교 가능한 형태로 만든다.

자동 기록

- standing time
- fall reason
- disturbance 조건
- controller parameter
- dt jitter
- run-to-run 비교 표

핵심 질문

실험 결과를 숫자로 비교하고 자동 보고서로 남길 수 있는가

현재 구현 범위(2026-03-14 기준)

- M8 raw를 읽는 후처리 extractor
- `balance_off_kpi.json`
- `balance_on_kpi.json`
- `comparison.json`
- `summary.md`
- `logs/sim2real/m9/index.csv`

---

# 7. 이후 확장 Milestone

포트폴리오 정리/문서화는 공식 milestone이 아니라 보조 작업으로 별도 관리한다.

아래 `M11+`는 "포트폴리오 증빙 순서"가 아니라,
실제 휴머노이드 제어 능력을 쌓아 가는 순서를 기준으로 다시 정의한다.

---

# M10 — Estimator / Observer 고도화 + Native Control Stack 고정

목표

센서 해석 / 제어 실행 / 하드웨어 경계를 명확히 고정하고,
그 위에서 estimator / observer를 고도화한다.

정리 대상

- `rb_estimation`
- `rb_standing_controller`
- `rb_hardware_interface`
- `rb_safety`

active path

`rb_estimation -> rb_standing_controller -> rb_hardware_interface -> rb_safety`

의도

- 이후 `M11+` milestone이 legacy controller 경로가 아니라
  native `ros2_control` 실행 경로를 기준으로 진행되게 만든다.
- `rb_controller`는 active runtime에서 제거하고 legacy/archive로 내린다.

구성

- IMU extrinsic calibration
- state estimation refinement
- contact-aware estimation

---

# M11 — Reactive Standing Envelope + Balance Metrics

목표

현재 `M5/M7/M8` standing baseline을 더 "바로 서 있는" 상태로 고도화하고,
정해진 외란 범위 안에서 복원 가능한 reactive standing을 만든다.

구성

- estimator / observer refinement
- disturbance envelope 정의
- COM projection
- ZMP
- capture point
- support polygon margin
- recovery time / peak tilt / settle time

핵심 질문

작은 외란에 대해서
"기우뚱거리며 겨우 버틴다"가 아니라
"upright posture를 다시 회복하며 선다"를 만들 수 있는가

---

# M12 — Standing Recovery Control (WBC / MPC / Recovery Step)

목표

joint-space stand PD에서 벗어나,
task-space / model-based standing recovery로 확장한다.

핵심 제어

- CoM
- torso
- foot
- foot placement
- recovery step trigger

가능한 접근

- WBC
- MPC
- reference generator + stabilizer

핵심 질문

더 큰 외란에 대해서
상체/발목만으로 버티는 것이 아니라,
필요하면 발을 옮겨가며 다시 설 수 있는가

---

# M13 — Walking / Locomotion

목표

recovery step을 연속 locomotion으로 확장한다.

구조

- gait / phase management
- step timing
- start / stop / turn
- velocity command tracking

핵심 질문

비상 복구용 한 발 내딛기를 넘어서,
의도적인 다중 스텝 보행을 만들 수 있는가

---

# M14 — RL Policy Layer

목표

low-level stabilizing controller / locomotion stack 위에
policy layer를 추가한다.

구조

low level stabilizing controller
→ high level policy / residual / reference

의도

- hand-designed stabilizer를 버리는 것이 아니라,
  그 위에 learned policy를 얹어 robustness / agility / command tracking을 넓힌다.

---

# M15 — Sim-to-Real Hardening + Experiment Infrastructure

목표

실험 결과를 체계적으로 비교 가능하게 유지하면서,
실기체 대응 robustness를 높인다.

추가

- actuator delay
- sensor noise
- latency
- domain randomization
- run comparison dashboard
- KPI trend plots
- failure taxonomy
- auto markdown summary

핵심 질문

걷기 / recovery controller가
깨끗한 sim 한 장면이 아니라 다양한 조건 변화에도 유지되는가

---

# M16 — Real-Time Execution Layer (RT Linux)

목표

control loop를 실제 로봇 환경에서 deterministic하게 실행하도록
real-time execution 환경을 구축한다.

핵심 작업

- PREEMPT_RT kernel 적용
- CPU isolation
- controller thread priority 설정
- ROS2 executor scheduling 분석
- control loop latency / jitter 측정

측정 항목

- control loop latency
- scheduling jitter
- dt variance (RT vs non-RT)

핵심 질문

제어 루프가 실제 로봇 환경에서도 deterministic하게 동작하는가

---

# M17 — Behavior / Task Planner + VLA Interface

목표

low-level controller / locomotion 위에
behavior / task planner 계층을 추가하고,
필요 시 VLA 같은 high-level task interface를 연결한다.

planner는 robot state / disturbance 상태 / task mode를 기반으로
action primitive를 선택하고 controller reference를 생성한다.

구조

planner
→ reference generation
→ controller
→ actuator

초기 planner 범위

- stand
- sit
- recover
- step preparation
- walk
- turn

후속 high-level 범위

- task mode selection
- semantic command to behavior mapping
- VLA / multimodal policy interface

핵심 질문

low-level controller 위에 behavior layer를 올려
전체 humanoid control loop를 구성할 수 있는가

---

# 8. 최종 아키텍처 비전

최종 시스템 구조

Sensors
(`/clock`, `/rb/joint_states`, `/rb/imu`)

→ Estimator

→ Planner / Behavior Layer

→ Controller

→ Safety

→ Actuator

→ Logger / Recorder

---

현재 active runtime 구조는 아래와 같다.

- `rb_interfaces`
- `rb_bringup`
- `rb_estimation`
- `rb_standing_controller`
- `rb_hardware_interface`
- `rb_safety`

즉 패키지 분리 자체는 이미 진행되었고,
이후 milestone은 이 native `ros2_control` 경로 위에서 확장한다.

최종적으로는 다음 ROS 노드 / 패키지 구조를 목표로 한다.

- Estimator
- Planner
- Controller
- Safety
- Logger

패키지 구조는 위 노드 구조와 같은 방향으로 가져가되,
한 번에 전부 분리하지 않고 milestone 진행에 맞춰 단계적으로 옮긴다.

권장 최종 패키지 구조

- `rb_interfaces`
- `rb_bringup`
- `rb_estimation`
- `rb_planning`
- `rb_standing_controller`
- 필요 시 `rb_control`
- `rb_safety`
- `rb_recorder`
- 필요 시 `rb_sim_bridge` / `rb_hardware_interface`

패키지 분리 시점 원칙

1. `M10`
   - native `ros2_control` 실행 경계를 고정한다.
   - 즉 `rb_estimation -> rb_standing_controller -> rb_hardware_interface -> rb_safety`
     경로를 active path로 확정한다.
2. `M11 ~ M13`
   - 위 active path를 유지한 채 estimation / balance / locomotion control을 고도화한다.
   - 필요 시 `rb_control` 또는 추가 controller package를 확장한다.
3. `M14 ~ M16`
   - RL / hardening / experiment infrastructure / RT 요구가 커지면
     `rb_bringup`, `rb_recorder`, 필요 시 `rb_sim_bridge`를 추가/분리한다.
   - 즉 launch / orchestration / logging / deployment 계층을 제어 패키지와 분리한다.
4. `M17`
   - planner / behavior / VLA interface가 실제 runtime 계층으로 들어오는 시점에
     `rb_planning`을 별도 패키지로 추가한다.

정리하면,

- 구조 설계는 지금 확정한다.
- 물리적 패키지 이동은 계약이 굳은 것부터 순서대로 한다.
- 아직 비어 있는 layer를 미리 패키지로 만드는 것보다,
  실제 runtime 책임이 생기는 시점에 분리하는 쪽을 기본 원칙으로 한다.

---

# 9. 현재 우선순위

현재 우선순위는 아래 순서로 고정한다.

1. `M11` reactive standing + balance metrics 범위 정의와 계약 정리
2. `M12` standing recovery control 방향 결정 (`WBC` / `MPC` / hybrid)
3. `M13` walking 진입 전제 정리
4. 문서·증빙 갱신과 small cleanup은 보조 작업으로 병행
