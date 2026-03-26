# STATUS — RB_Humanoid_Control

## 1. 현재 프로젝트 한 줄 요약
휴머노이드 제어 엔지니어 포지션 지원을 위해, 실기체 없이 Isaac Sim + ROS2 기반 휴머노이드 제어 시스템을 직접 구성하고 증빙하는 프로젝트.

## 2. 현재 결정사항
### 유지
- 로봇 모델은 G1 유지
- Gym/IsaacLab Stage1 코드는 baseline/archive로 유지
- 실행 진입점은 루트 `main.py` 유지

### 폐기
- `saved USD`를 메인 source of truth로 쓰는 방식 폐기
- Lab에서 한 번 돌리고 저장한 USD를 메인 자산으로 쓰지 않음

### 채택
- 메인 경로는 `원본 G1 asset/cfg direct spawn + standalone World.step()`
- `saved USD`는 debug/reference artifact로만 사용
- M5는 disturbance 기준 baseline/stand 비교로 재정의

## 3. 완료된 것
### 구조/아키텍처
- ROS2 메인 트랙을 standalone `World.step()` 기준으로 재정리
- mixed command path 정리(effort-only 경로 정리)
- tmuxp 기반 재검증 플로우 정리
- standalone backend code path를 `isaaclab_assets.robots.unitree.G1_CFG` direct spawn 기준으로 전환

### standalone 재검증
- M1 완료
  - `/clock`, `/rb/joint_states`, `/rb/imu` 확인
- M2 완료
  - controller 200Hz loop / `/rb/command_raw` 확인
- M3 완료
  - command apply / joint state 변화 확인
- M4 완료
  - `CLAMP`, `JOINT_LIMIT`, `TIMEOUT`, `TILT` 개별 재검증 완료

### 최신 리팩 완료 (2026-03-23)
- ROS control stack을 `Estimator -> Controller -> Safety` 런타임 경계로 재정리했다. `rb_estimator_node`를 분리했고, `/rb/estimated_state`를 도입해 controller와 safety가 같은 추정 상태를 기준으로 동작하도록 맞췄다.
- safety는 raw IMU direct tilt 해석 대신 estimated state를 사용하도록 정리했다. controller 내부도 `controller_stand_utils.*`, `controller_runtime_state.hpp` 기준으로 helper/runtime cache를 분리해 node 책임을 더 얇게 만들었다.
- Isaac app은 `main -> phase_registry -> phase_handlers -> phase_runtime` 구조로 phase orchestration을 분리했고, world(`world_factory/runtime/spawn`), bridge(`graph_prim_resolver`, `sensor_graph_builder`, `command_apply_graph_builder`), KPI(`kpi/parsers/model/writers`) 레이어를 나눴다. 구조화 로그는 `telemetry.py`로 모았다.
- 기존 엔트리 경로는 유지했다. `main.py`, `controller.launch.py`, `ops/run_m8_and_m9.sh`, `extract_m8_kpi.py`는 그대로 쓰되 내부 구현만 facade + 분리 모듈 구조로 정리했다.
- 검증은 `colcon build --packages-select rb_controller`, Python syntax check, `python main.py --phase m1_sensor --steps 2 --headless`, `ops/run_m8_and_m9.sh` smoke까지 통과했다. smoke 중 드러난 `extract_m8_kpi.py` import 경로 문제와 `kpi/writers.py`의 short-run `None` 포맷 문제도 수정했다.
- 주의: 아래 구간에 남아 있는 `estimator 분리는 M10에서 진행` 같은 문구는 리팩 이전 메모다. 현재는 경계 분리는 완료됐고, 다음 M10의 초점은 estimator 내부 refinement와 contract 안정화다.


## 4. 최근 조사 결과
### IsaacLab asset reference
- `/home/leemou/IsaacLab/source/isaaclab_assets/isaaclab_assets/robots/unitree.py` 기준 `G1_CFG -> Robots/Unitree/G1/g1.usd` direct spawn 경로 확인

### direct spawn bring-up
- minimal sanity(`create_standalone_world() + world.step()`) 기준 direct spawn 1-step 통과 확인
- phase runtime 기준 asset load + graph build는 통과했지만, `RUNNING` 이후 첫 simulation step 진행은 추가 확인 필요
- Isaac Sim 로그에서 `humble.rclpy` import warning 관찰됨
- `MILESTONE_REVALIDATION_GUIDE.md`를 direct spawn + `conda activate isaac` 기준으로 갱신
- `VIDEO_CAPTURE_GUIDE.md` 추가: milestone별 raw clip 수집 + final edit 기준 정리
- `VIDEO_CAPTURE_GUIDE.md` 보강: disturbance spec 고정, M3 visible motion 우선, M5-first final hook, KPI ending 문구 추가
- `VIDEO_CAPTURE_GUIDE.md` 추가 보강: split-screen 기준, elapsed-time overlay, final one-line message 고정
- `VIDEO_CAPTURE_GUIDE.md` 최상단에 90초 포트폴리오 영상 시나리오 추가
- `VIDEO_CAPTURE_GUIDE.md` 보강: milestone별 Isaac 화면 / terminal 화면 / 권장 비중 명시
- `VIDEO_CAPTURE_GUIDE.md` 재정리: `M1~M4` terminal-heavy, Isaac은 `서 있음 / 명령 반응 / disturbance 비교` 3종 장면 중심으로 축소
- `VIDEO_CAPTURE_GUIDE.md` 단순화: 현장 촬영 체크리스트 형태로 재작성, milestone별 필수 화면만 남김
- `ROADMAP_SIMPLE.md` 재정리: 각 milestone을 `무엇을 하는가 / 왜 하는가 / 목표` 기준으로 단순화하고, M5 실행 순서를 별도 섹션으로 정리
- `M3` 촬영/실행 기준 재조정: `step_effort + left_shoulder_roll_joint`로 arm raise가 먼저 보이게 변경
- `M3` timing/출력 추가 조정: 고정 15초 대기를 제거하고 `/clock` 직후 `step_start_sec=0.5`, `effort_amplitude=4.0`으로 onset을 앞당김
- `M3` target joint 재조정: 수동 확인으로 움직임이 분명한 `left_shoulder_pitch_joint` 기준으로 변경
- `VIDEO_CAPTURE_GUIDE.md` 최종 시나리오 고정: `M5 split-screen hook -> M1~M4 terminal highlight -> M5 replay` 구조로 단순화
- `M5` 운영 세션 보강: `ops/tmuxp/m5_stand.yaml` 추가, `baseline_zero`와 `stand_pd_default`를 둘 다 tmuxp로 재현 가능하게 정리
- `M5` no-disturbance sanity 경로 추가: `stand_pd_default`가 baseline과 동일하게 약 3.5초 내 붕괴해 disturbance 이전에 `stand` 자체를 먼저 살려야 하는 상태로 판단
- `stand_pd_sanity.yaml` / `m5_stand_sanity.yaml` 추가: 하체+몸통만 제어, `tilt_feedback=off`, `stand_tilt_cut=off`, `stand_kp=30`, `stand_kd=2`, `stand_effort_abs_max=6` 기준 no-disturbance stand 전용 시나리오 분리
- `M5` tmuxp pane 3를 고정 `sleep 80` 대신 `첫 safety reason 감지 -> 즉시 reason_count/loop_tail` 방식으로 변경하고, 60초 내 이벤트가 없으면 fallback summary를 생성하도록 조정
- 위 이벤트 기반 pane 3 명령의 YAML quoting 오류를 수정해 `tmuxp load`가 다시 정상 동작하도록 정리
- `m5_stand_sanity` 1차 관측에서 `pitch=1.004`로 즉시 `reason=TILT`가 발생해, stand gain 평가 전에 safety가 먼저 끊는 상태로 판단. `stand_pd_sanity.yaml`의 safety tilt limit을 `roll=1.1`, `pitch=1.3`으로 완화
- 위 완화 후에도 `pitch=1.302`에서 다시 즉시 `reason=TILT`가 발생해, no-disturbance stand bring-up 단계에서는 `stand_pd_sanity.yaml`의 safety TILT 한계를 `roll=3.14`, `pitch=3.14`로 사실상 비활성화
- 이후에도 `roll=-3.140`, `pitch=1.491`처럼 upright와 맞지 않는 IMU tilt가 보여, `stand_pd_sanity.yaml`에서는 TILT safety 한계를 `roll=10.0`, `pitch=10.0`으로 올려 stand PD pose-hold와 IMU/frame 문제를 분리해서 보기로 함
- `TILT safety`를 사실상 제거한 뒤에도 no-disturbance stand가 약 `3초`로 동일하게 붕괴해, 2차 튜닝으로 `stand_kp=60`, `stand_kd=4`, `stand_effort_abs_max=12`, `hip/knee/ankle/torso` scale 상향을 적용
- 2차 튜닝 후에는 `reason=CLAMP`가 반복돼 safety clamp가 controller authority를 먼저 자르는 상태로 확인됨. `stand_pd_sanity`는 controller-only bring-up 단계로 정의하고, 여기서는 `rb_safety.safety_enabled=false`로 pure gain tuning을 진행한 뒤 safety를 다시 켜서 재통합하기로 함
- controller/safety 모두에 `imu_zero_on_start`를 추가해 첫 IMU 샘플을 bias로 캡처하고 이후 `roll/pitch - startup_bias`를 사용하도록 변경. `stand_pd_sanity`에서는 이 영점 보정을 켜서 `tilt_r/tilt_p`가 0 근처로 내려오는지 먼저 확인하기로 함
- `stand_pd_sanity`의 `current pose hold`를 버리고, IsaacLab `G1_CFG.init_state.joint_pos`를 `joint_order_g1.yaml` 순서에 맞춘 full-length `stand_q_ref` seed로 고정. helper `scripts/sim2real/generate_g1_stand_qref.py` 추가
- graph_builder의 IMU source를 `robot_prim=/pelvis` 고정에서 분리. `imu_prim` 자동탐색(`imu_link -> torso_link -> pelvis`)과 `frame_ids.imu=auto`를 추가해 `/rb/imu`가 실제 source prim 이름과 맞는 frame_id를 사용하도록 조정
- `joint sign`을 육안 대신 숫자로 확인하기 위한 최소 audit 경로 추가: `joint_sign_audit.yaml` + `ops/tmuxp/joint_sign_audit.yaml` + `scripts/sim2real/joint_state_delta.py`. 단일 joint에 작은 `step_effort`를 주고 `js_before/js_after`에서 `delta` 부호를 계산하도록 정리
- 하체 6개 sign audit 재확인 결과 hip/knee/ankle pitch는 모두 `+ command -> + delta`, `- command -> - delta`로 1차 통과. 다음 단계로 `stand_pd_sanity`에서 `enable_tilt_feedback=true`를 다시 켜고 standing time 변화 확인
- pose audit 결과 `IsaacLab G1` seed pose가 팔 전방/상대적 crouch로 인해 free-stand 시 항상 앞으로 붕괴하는 패턴으로 해석됨. `standing q_ref v2`로 hip/knee/ankle crouch를 줄이고 팔을 내린 nominal pose로 보정
- `m5_stand`에서는 controller target뿐 아니라 Isaac direct spawn 초기 pose도 같은 `stand_qref_g1_seed.yaml`를 사용하도록 변경. 이제 시작 pose와 `stand_q_ref` seed가 일치한 상태에서 stand sanity를 볼 수 있음
- 위 seed pose를 실제 spawn 화면에도 반영되게 수정. standalone world에서 `sim.reset()` 직후 `robot.data.default_root_state/default_joint_pos`를 직접 sim에 써서, `spawn_joint_seed_path`가 버퍼뿐 아니라 실제 초기 관절 자세에 적용되도록 정리
- `m5_pose_audit` phase/tmuxp 추가: M5 pose 확인 전용으로 `start_paused=true`, `graph_builder.enabled=false`, controller 미실행 상태에서 GUI만 띄우는 경로를 분리. `main.py --phase m5_pose_audit`는 timeline을 play하지 않고 render-only idle loop로 유지하므로, pose 편집 시 동적 joint state 대신 정지 상태 값을 읽을 수 있게 함
- `m5_pose_audit`에 `startup_joint_dump_path` 추가. GUI Property 패널의 `Joint State` 숫자는 paused 상태에서도 runtime cache 영향이 있어 source of truth로 부적절하므로, startup 직후 `robot.data.default_joint_pos`를 `logs/.../m5/startup_joint_pos.yaml`로 저장해서 nominal pose seed 확인은 이 파일을 기준으로 하도록 정리
- pose audit dump에 `startup_joint_vel`, `named_joint_vel`, `startup_root_linvel_xyz`, `startup_root_angvel_xyz`를 추가. startup nominal pose뿐 아니라 시작 직후 velocity가 이미 0이 아닌지까지 같은 파일에서 바로 확인할 수 있게 함
- `ops/tmuxp/m5_pose_audit.yaml`의 안내 문구 한 줄이 YAML에서 dict로 파싱돼 `tmuxp load`가 실패하던 quoting 오류를 수정
- `m5_pose_audit` dump에서 startup joint/root velocity가 전부 0으로 확인돼, spawn 직후 이미 동적으로 깨지는 문제는 아니라고 판단. 다음 단계로 nominal standing pose v3를 더 upright하게 보정(`hip_pitch=-0.05`, `knee=0.15`, `ankle_pitch=-0.08`, `shoulder_pitch=-0.30`, `elbow_pitch=0.0`, `shoulder_roll=0.0`)하고, `spawn_root_height_z=0.78`을 추가해 시작 자세 자체를 덜 crouched하게 조정
- `m5_pose_audit`에서 GUI로 upper-body pose를 직접 확인할 수 있게 arm actuator override를 분리. pose audit 경로에서는 `G1_CFG.actuators["arms"]`의 `stiffness/damping`만 `0/0`으로 낮추고, main standing 경로(`m5_stand`, `m5_stand_sanity`)는 기존 actuator 설정을 유지
- 위 `0/0` arm override는 pose audit에서 팔이 자유 낙하처럼 출렁여 upper-body pose 확인에 오히려 방해가 돼, `m5_pose_audit` arm actuator를 soft-hold(`stiffness=8.0`, `damping=1.5`)로 조정
- GUI pose audit 기준으로 upper-body nominal pose를 다시 고정: `left/right_shoulder_pitch_joint≈0.26`, `left/right_elbow_pitch_joint≈0.35`. spawn seed와 `stand_q_ref`가 같은 upper-body pose를 쓰도록 seed generator와 `stand_pd_sanity`를 동기화
- upper-body pose 미세조정: GUI 확인 결과 `left/right_elbow_pitch_joint`는 `0.35`보다 `0.50` 근처가 더 자연스러워 보여, spawn seed와 `stand_q_ref`의 elbow pitch를 `0.50`으로 상향
- `stand_pd_sanity`의 tilt stabilization 구조를 `effort injection`에서 `q_ref bias` 방식으로 전환 시작. controller는 `tilt_apply_mode=qref_bias`일 때 IMU 기반 보정값을 각 joint effort에 직접 더하지 않고, nominal `stand_q_ref`를 ankle/hip/knee/torso 쪽으로 소폭 이동시킨 뒤 기존 joint PD가 그 자세를 따라가게 함

### zero-drive saved USD 조사
- `g1_stage_zero_drive.usd` 생성 및 확인 수행
- joint drive audit 결과:
  - stiffness = 0
  - damping = 0
  - target_pos = 0
  - target_vel = 0
- 즉, 기본 spring hold 자체는 제거된 상태로 보임

### dynamic audit 결과
- rigid body: 정상
- kinematic enabled: 없음
- gravity disabled: 없음
- root를 바닥에 고정하는 fixed joint: 확인되지 않음

### 현재 해석
- saved USD 경로는 물리 의미를 계속 해석해야 하는 비용이 큼
- direct spawn 경로로 source of truth를 옮기는 것이 더 낫다고 판단

## 5. 현재 블로커
### 최우선 블로커
- direct spawn phase 경로에서 첫 simulation step stall 원인 확인(graph/ROS2 포함)
- M1~M4 quick regression

### 그 다음 블로커
- disturbance 기준 baseline 정의 확정
- `baseline_zero` vs `stand_pd_default` 비교 실험 확정
- no-disturbance `stand`를 먼저 확보하기 위한 stand sanity tuning

## 6. 다음 작업
1. direct spawn phase 경로 stall 원인 분리(graph vs ROS2 bridge vs runtime env)
2. M1~M4 quick regression
3. M5 no-disturbance stand sanity 확인
4. M5 disturbance A/B 비교

## 7. 포트폴리오 관점 메모
현재 중요한 것은 모델 이름이 아니라 아래 증빙이다.
- ROS2 control path
- disturbance A/B
- safety
- dt-jitter
- KPI
- 로그/캡처/영상

즉 `Atlas + 미완성`보다 `G1 + 완성된 제어 증빙`이 훨씬 낫다.

## 8. 리스크
- direct spawn 경로에서 prim path가 달라질 수 있음
- asset spawn 후 actuator/drive 특성이 다시 보일 수 있음
- 따라서 direct spawn 전환 후 M1~M5 일부를 다시 확인해야 함

## 9. 현재 판단
- Atlas 전환은 보류
- G1 유지
- original G1 direct spawn standalone 채택
- disturbance A/B + KPI summary까지 포함해 3/18~3/19 지원 목표
- `stand_pd_sanity` 1회 추가 실험: `u_pitch`가 `-1.800`에서 지속 포화되어 있어 `tilt feedback authority`가 병목으로 판단. 다음 런은 `tilt_kp_pitch=18`, `tilt_kd_pitch=3`, `tilt_effort_abs_max=4.0`, `tilt_weight_pitch(h/a/k/t)=0.9/0.6/0.15/0.05`로 발목/엉덩이 중심 pitch 보정 상향.
- `standing q_ref v4` 적용: sagittal chain을 더 upright하게 재설정(`hip_pitch=-0.02`, `knee=0.08`, `ankle_pitch=-0.03`)하고 upper-body pose(`shoulder_pitch=0.26`, `elbow_pitch=0.50`)는 유지한 채 `spawn seed`와 `stand_q_ref`를 다시 동기화. 목적은 `3초 전방 붕괴`가 nominal pose 문제인지 더 강하게 분리 확인하는 것.
- `rb_controller loop_stats`에 `torso/hip_pitch/knee/ankle_pitch ref/meas`와 `pitch_bias(hip/knee/ankle/torso)` observability를 추가. 다음 `m5_stand_sanity` 런에서는 `qref_bias`가 실제 목표각을 얼마나 움직였고 joint가 따라가는지를 `loop_tail`만으로 바로 확인한다.
- `stand_pd_sanity` 오케스트레이션을 safety-event wait에서 short-window capture로 전환. `rb_safety.safety_enabled=false` 상태에서 upright `q_ref v4`가 초기 `3~8초` 붕괴 패턴을 바꾸는지만 보도록 `log_interval_sec=1.0`, launch timeout `15s`, pane 3 capture delay `8s`로 재정렬했다.
- `stand_pd_sanity` capture trigger를 `FALL_EVENT` 기준으로 재전환. Isaac pane에서 `pelvis/root` 또는 `torso_link` 높이가 threshold 이하(`pelvis_z<=0.45`, `torso_z<=0.58`)로 내려가면 `[FALL_EVENT]`를 찍고, pane 3는 그 이벤트를 기다렸다가 `fall_event.txt`와 `loop_tail.txt`를 수집한다.
- `FALL_EVENT` threshold를 더 보수적으로 조정(`pelvis_z<=0.30`, `torso_z<=0.40`)하고, `stand_pd_sanity` controller 로그 간격을 `0.2s`로 높여 `loop_early.txt`로 초기 2초 안쪽 pre-fall 변화를 더 촘촘하게 보도록 변경.
- `loop_stats`에 ankle effort observability를 추가. 다음 `m5_stand_sanity` 런에서는 `ankle_pd_avg`, `ankle_lim_avg`, `ankle_pre_avg`, `ankle_cmd_avg`, `ankle_sat`로 발목에 주려던 힘과 clamp 여부를 바로 확인한다.
- ankle stand PD saturation 완화 실험으로 `stand_kp_scale_ankle=2.0`, `stand_kd_scale_ankle=1.5`로 낮췄다. 목적은 발목이 `±12` clamp에 계속 박는 현상이 줄어드는지 먼저 확인하는 것이다.
- 위 조정만으로는 `fall_event`가 여전히 `1.5s`라 효과가 제한적이었다. 다음 실험으로 `stand_effort_abs_max=16.0`만 상향해, 발목이 `±12` ceiling 때문에 막히는지 분리 확인한다.
- `stand_effort_abs_max=16.0`에서도 `fall_event`가 계속 `1.5s`로 찍혀 timing 해상도가 부족하다고 판단했다. `m5_stand` fall trigger의 `min_elapsed_sec`를 `0.8`로 낮춰 실제 붕괴 시점이 조금이라도 늦어졌는지 더 민감하게 본다.
- `standing q_ref v5` 실험으로 sagittal chain을 중간값으로 되돌렸다(`hip_pitch=-0.04`, `knee=0.12`, `ankle_pitch=-0.05`). 목적은 너무 upright한 `v4`가 초반부터 발목에 큰 부담을 주는지 확인하는 것이다.
- `standing q_ref v6`로 IsaacLab 공식 `G1_CFG.init_state` baseline을 적용한다. `hip_pitch=-0.20`, `knee=0.42`, `ankle_pitch=-0.23`, `shoulder_pitch=0.35`, `shoulder_roll=±0.16`, `elbow_pitch=0.87`, `spawn_root_height_z=0.74`를 그대로 사용해 추측값 대신 공식 기준점으로 재검증한다.
- 공식 `v6` pose에서도 `fall_event≈1.2s`라 pose만으로는 부족하다고 판단했다. 다음 실험으로 `m5_stand`에서 ankle actuator만 `stiffness=40`, `damping=4`, `effort_limit_sim=40`으로 상향해 실제 actuator authority 병목인지 분리 확인한다.
- ankle actuator 1차 강화(`40/4/40`)에서 `fall_event=1.555s`로 개선이 보여 같은 방향으로 2차 실험을 진행한다. `stiffness=60`, `damping=6`, `effort_limit_sim=40`으로 한 단계 더 상향해 추가 이득과 진동 여부를 본다.
- ankle actuator 2차 강화(`60/6/40`)에서 `fall_event=1.735s`까지 늘어 actuator 방향이 유효하다고 판단했다. 다음 실험으로 controller ceiling 분리를 위해 `stand_effort_abs_max=20.0`으로 상향한다.
- `stand_effort_abs_max=20.0`은 `1.620s`로 오히려 약간 악화됐다. 다음 실험으로 중간값 `18.0`을 적용해 `16`과 `20` 사이의 effort ceiling 최적점을 확인한다.
- `tilt` 파라미터 재조정보다 actuator 쪽이 더 유효하다고 판단해, controller는 현 상태를 유지한 채 ankle actuator 3차 강화(`stiffness=80`, `damping=8`, `effort_limit_sim=40`)를 시험한다.
- 현재 best baseline은 official `v6` pose + ankle actuator `80/8/40` + `tilt_qref_bias_abs_max=0.15` 기준으로 둔다. controller ceiling은 `16~20` 범위 차이가 작아 우선 `18.0`을 기준값으로 고정한다.
- 다음 1순위 실험은 `tilt_kp_pitch`만 `18.0 -> 20.0`으로 올려, 발목 actuator 강화 이후 남은 병목이 pitch 보정 강도인지 확인하는 것이다.
- `tilt_kp_pitch=20.0`에서는 headless 기준 `fall_event≈1.86s`까지 늘었지만, 아직 `tilt_p`가 단조 증가한다. 다음 실험으로 `tilt_kd_pitch=2.0`만 낮춰 pitch-rate 과민 반응을 줄이고 hold time 변화를 본다.
- 최근 GUI/로그 기준으로는 `tilt_p≈0`, `u_pitch≈0`, `ankle_sat=0`인데 `tilt_r`와 `hip_yaw/ankle_roll` 오차가 남아 roll 쪽 불안정이 더 우세해 보인다. 다음 실험은 roll 보정 분배를 기본값 `hip 0.7 / ankle 0.3`에서 `hip 0.5 / ankle 0.5`로 옮겨, 옆방향 복원을 발목에 더 싣는 것이다.
- GUI 재확인 결과 실제 거동은 여전히 전방 붕괴가 우세했고, roll 분배 변경은 효과가 없었다. 다음 실험은 roll 분배를 원복한 뒤 `stand_kp_scale_ankle: 2.0 -> 2.5`만 올려 발목 pitch tracking authority를 소폭 강화한다.
- 최근 반복 실행 결과, GUI에서는 같은 세팅에서도 `fall_event≈1.6s`와 `≈1.9s`가 섞여 startup 편차가 크지만, headless에서는 같은 세팅이 `≈1.63s` 근처로 거의 고정된다. 따라서 앞으로의 정량 비교 baseline은 headless로 고정하고, GUI는 넘어지는 방향/형태를 눈으로 확인하는 용도로만 사용한다.
- 현재 판단상 `ankle_actuator_override(80/8/40)` 이후에는 발목 authority 부족보다 `headless 1.63s`에서 반복되는 전방 붕괴 모드 자체가 다음 병목이다. 즉 gain 미세조정보다 실패 모드를 바꾸는 더 구조적인 원인 분리가 다음 단계다.
- `pitch correction distribution` 재분배(`hip 1.1 / ankle 0.4 / knee 0.25`)는 headless 기준 `≈1.63s`로 baseline과 차이가 없었고, `tilt_p≈0`, `u_pitch≈0`, `pitch_bias≈0`로 pitch 보정 자체가 약해지는 방향으로 보였다. 이 브랜치는 기각하고 baseline(`hip 0.9 / ankle 0.6 / knee 0.15`)으로 롤백한다.
- 다음 실험은 total pitch 보정량은 유지한 채 발목 분담만 소폭 상향하는 것이다. `tilt_weight_pitch_hip: 0.9 -> 0.8`, `tilt_weight_pitch_ankle: 0.6 -> 0.7`, `tilt_weight_pitch_knee=0.15`, `torso=0.05`로 바꿔, hip 대신 ankle가 전방 붕괴를 조금 더 직접 받치게 한다.
- safety node 보강 작업을 시작했다. `TIMEOUT`를 실제 `/rb/command_raw` watchdog으로 바꾸고 `VELOCITY_LIMIT`를 추가했으며, 튜닝 기본 safety 수치는 `input_timeout_sec=0.3`, `tilt_limit_roll/pitch=0.6 rad`로 정리한다. GUI/headless stand 튜닝과는 분리해, 이후 safety 재통합 기준선으로 사용한다.
- headless standing에서 실제로는 전방 붕괴인데 `tilt_p≈0`, `u_pitch≈0`가 반복돼 IMU pitch 관측/zeroing 의심이 커졌다. `controller loop_stats`에 `imu_raw_r/p`와 `imu_bias_r/p`를 추가해 raw quaternion pitch와 bias 제거 후 tilt가 어떻게 다른지 직접 보이게 한다.
- raw/bias 로그 확인 결과 forward fall이 `pitch`가 아니라 현재 `roll` 채널 변화로 들어오고 있었다(`imu_raw_r`만 유의미하게 변하고 `imu_raw_p`는 거의 고정). 다음 실험은 `stand_pd_sanity`에서만 `tilt_axis_mode=swap_rp`를 적용해, forward 붕괴를 pitch 보정 루프가 실제로 보게 만든다.
- `m5_stand_sanity` tmux 오케스트레이션의 고정 `sleep 15`는 제거했다. GUI/headless startup 차이 때문에 controller launch/capture 시점이 흔들릴 수 있어, 이제는 controller를 즉시 띄우고 sync marker 기반으로 분석한다.
- log 분석 방식도 정리한다. `m5_stand_sanity`는 이제 controller를 즉시 띄우고, 원본 로그 전체(`isaac.log`, `m5_stand_sanity.log`)를 보존한 채 `[SYNC] FIRST_SIM_STEP`, `[SYNC] CONTROL_ACTIVE`, `[FALL_EVENT]` marker를 기준으로 `sync_markers.txt`, `loop_post_sync.txt`, `loop_before_fall.txt`를 후처리 추출한다.
- `m5_stand` 실행 종료 조건도 고정한다. 기본은 `stop_on_fall_event=true`로 두고, `FALL_EVENT`가 찍히면 rollout step loop를 즉시 종료한다. `m5_stand_sanity` tmuxp는 더 이상 고정 30초에 잘라먹지 않고 `CAPTURE_MAX_SEC` fallback만 둔 채 `FALL_EVENT` 기준으로 로그를 추출한다.
- `m5_stand_sanity` tmuxp는 `AUTO_KILL_AFTER_CAPTURE=1`일 때 capture 완료 후 세션을 자동 종료한다. 현재는 `FALL_EVENT` 유무와 무관하게 `FIRST_SIM_STEP + 고정 관측창` 이후 파일 추출이 끝나면 `tmux kill-session`으로 정리한다.
- standing이 길어져 `FALL_EVENT`가 안 뜨는 경우를 대비해 `m5_stand_sanity` 캡처 기준을 다시 바꿨다. 이제 pane 3는 `[SYNC] FIRST_SIM_STEP`를 기준으로 `POST_SYNC_CAPTURE_SEC`(기본 60초) 관측창을 잡고, 그 시간이 지나면 fall 유무와 관계없이 `fall_event/sync_markers/loop_post_sync/loop_before_fall`를 추출한 뒤 세션을 종료한다. `fall_event.txt`에는 fall이 없으면 `[NO_FALL_EVENT]`를 남긴다.
- 구조 리팩터링 방향도 고정한다. 최종 목표는 `Estimator/Observer`, `Controller`, `Safety`, `Logger/Recorder`의 4개 노드 분리이지만, 현재 standing tuning 단계에서는 런타임 변수를 늘리지 않기 위해 노드 분리 대신 `rb_controller` 내부를 `stand core / tilt observer / debug logger` 모듈로 먼저 나누는 점진 리팩터링을 우선한다.
- 1차 모듈화 완료: `rb_controller` 내부에서 IMU raw/bias/axis remap 처리는 `controller_tilt_observer.*`, sync/loop 문자열 포맷은 `controller_debug_logger.*`, 공용 스냅샷 타입은 `controller_types.hpp`로 분리했다. 런타임 동작/토픽 구조는 그대로 유지하고 `controller_node.cpp`는 ROS orchestration + stand core 중심으로 얇게 만들기 시작했다.
- 장시간 hold 실험에서 `swap_rp` 적용 시 `60~90초` 이상 no-fall이 반복돼, standing 실패의 핵심 원인은 gain 부족보다 `imu_link -> control frame` 해석 불일치로 판단한다. raw `/rb/imu` publisher는 그대로 두고 observer 쪽에서 `imu_frame_mode=g1_imu_link` 보정을 적용하는 방향을 새 baseline으로 채택한다.
- 원인/해결을 더 명확히 고정한다. 기존 controller는 `imu_link` raw orientation을 이미 body/control frame에 정렬된 roll/pitch처럼 직접 사용해, 실제 전방 붕괴가 `pitch`가 아니라 `roll` 채널 변화로 들어오고 있었다. 해결은 IMU publisher를 뒤집는 것이 아니라 observer에서 `imu_link -> control frame` 보정을 적용하는 것이며, 현재 `imu_frame_mode=g1_imu_link`가 그 역할을 한다.
- 문서 운영 기준도 정리했다. `README.md`는 일반 사용 설명서가 아니라 포트폴리오 랜딩 페이지로 유지하고, milestone 의미가 바뀔 때마다 `README + reports/sim2real/overview.md + reports/sim2real/ONE_PAGER.md + STATUS.md`를 같이 동기화한다.
- README/overview/ONE_PAGER도 현재 상태에 맞춰 갱신했다. README는 포트폴리오 랜딩 페이지 톤으로 한글 위주 재작성했고, milestone별로 왜 이 순서로 진행했는지/무엇을 구현했고/무엇으로 검증했는지와 현재 standalone 증빙 링크를 반영했다.
- `MASTER_PLAN.md`도 다시 milestone형 구조로 정리했다. 기존 `M0~M6` 흐름은 유지하고, 뒤에 safety 재통합, disturbance A/B, KPI 자동화, estimator/COM-ZMP/WBC/RL policy 등 후속 milestone을 추가했다.
- `MASTER_PLAN.md`를 최종 milestone 문서로 다시 다듬었다. `M5`에는 실제 standing 실패 원인이었던 `imu_link -> control frame` 해석 불일치와 observer 보정 해결을 명시했고, `M6`는 증빙/아티팩트 인프라, `M9`는 KPI 자동 요약/비교로 역할을 분리했다.
- `MASTER_PLAN.md`를 한 번 더 다듬어 현재 실험 상태와 맞췄다. `M6` 출력 구조를 실제 `logs/sim2real/<milestone>/<run_id>` 운영 기준으로 맞추고, `M8`은 아직 진행 중인 disturbance milestone답게 `목표 결과`와 `1차 sagittal / 후속 lateral` 구분으로 정리했으며, 후속 확장에는 `M16 RT Linux`, `M17 Behavior / Task Planner`를 추가했다.
- `ROADMAP_SIMPLE.md`도 `MASTER_PLAN.md`와 같은 milestone 체계로 다시 정리했다. 현재 standing 성과(`imu_frame_mode=g1_imu_link`)와 다음 우선순위(`M7 -> M8 -> M9`)가 한눈에 보이게 정리했다.
- `AGENTS.md`의 문서 운영 규칙도 보강했다. `overview`는 milestone 완료/원인 해결 직후, `README`는 외부 공개용 상태 변화 시, `ONE_PAGER`는 `M5/M8/굵직한 체크포인트` 기준으로 갱신하도록 타이밍 기준을 명시했다.
- `ROADMAP_SIMPLE.md`의 milestone 제목도 한글 중심으로 정리했다. `M0 인터페이스 고정`, `M5 스탠딩 안정화`, `M8 외란 A/B 비교`처럼 한눈에 훑기 쉬운 형태로 맞췄다.
- `ROADMAP_SIMPLE.md`를 표 중심 요약 문서로 다시 배치했다. `M0~M6`, `M7~M9`, `M10+`를 각각 표 1개로 묶어 `무엇 / 왜 / 목표`를 칸 안에서 바로 비교할 수 있게 정리했다.
- `M7` safety 재통합용 경로를 분리했다. `stand_pd_safecheck.yaml`은 현재 standing baseline에 `safety_enabled=true`, `effort_abs_max_default=18.0`, `tilt_limit=0.6rad`, `velocity_limit(default/ankle)=8/12rad/s`를 얹은 전용 시나리오이고, `ops/tmuxp/m7_stand_safecheck.yaml`은 headless 60초 관측창 동안 `reason_count/sync_markers/loop_post_sync/loop_before_fall`를 자동 수집한다.
- `VIDEO_CAPTURE_GUIDE.md`도 현재 milestone 체계로 다시 맞췄다. 예전 `M5 disturbance hook` 중심 구조를 버리고, `M5 standing hold -> M7 safety-on -> M8 disturbance A/B -> M9 KPI` 순서로 어떤 장면을 찍고 어떻게 편집할지 기준을 다시 정리했다.
- `M7` safecheck 캡처 기준도 조정했다. `m7_stand_safecheck.yaml`은 이제 `FIRST_SIM_STEP`가 아니라 controller 로그의 `[SYNC] CONTROL_ACTIVE`를 관측창 시작점으로 삼아, “sim이 시작된 뒤 60초”가 아니라 “실제 제어가 붙은 뒤 60초”를 기준으로 safety-on standing을 해석한다.
- `M4` safety 증빙도 보강한다. `velocity.yaml` + `ops/tmuxp/m4_velocity.yaml`을 추가해 팔 관절 하나를 빠르게 움직이는 전용 시나리오로 `reason=VELOCITY_LIMIT`를 재현하고, `reasons.txt`와 `loop_tail.txt`를 기존 M4 reason과 같은 방식으로 수집한다.
- `M7`은 safety-on standing 기준으로 사실상 통과했다. `logs/sim2real/_legacy/20260314-133954_m7_stand_safecheck/m7/`에서 `CONTROL_ACTIVE` 이후 60초 동안 `[NO_FALL_EVENT]`, `[NO_SAFETY_REASON]`를 확인했고, 이에 맞춰 `README/overview/ONE_PAGER/VIDEO_CAPTURE_GUIDE`를 M8 진입 전 상태로 다시 동기화했다.
- `MASTER_PLAN.md`와 `overview.md`의 M5 설명도 보강했다. 현재 standing controller는 raw joint/IMU direct feedback 기반의 경량 bring-up 구조이며, estimator 분리는 후속 milestone(M10)에서 진행한다는 점을 명시해 “왜 estimator가 아직 별도 노드가 아닌가”를 문서에서 바로 설명할 수 있게 했다.
- `M8` 비교 경로도 시작했다. `m8_disturb` phase에 torso 단일 force pulse 훅(`DISTURBANCE_START/END`)을 추가하고, `stand_pd_balance_off.yaml`(tilt/balance feedback off) / `stand_pd_balance_on.yaml`(현재 baseline) / `ops/tmuxp/m8_disturb.yaml`을 만들어 같은 외란에서 balance feedback OFF/ON을 같은 방식으로 수집할 수 있게 정리했다.
- 위 M8는 “외란 유무 비교”가 아니라 “같은 torso push에서 balance feedback 유무 비교”라는 의미로 고정한다. `m8_disturb.yaml`은 `loop_after_disturb.txt`와 함께 `disturb_kpi.txt`를 생성해 `peak_abs_tilt_r/p_after_disturb`를 no-fall 상황에서도 비교 가능하게 남긴다.
- 첫 M8 런에서는 `120N x 0.20s` torso push가 너무 강해 `balance_off`와 `balance_on` 모두 외란 직후 `~5.8s`에 fall했다. 이에 따라 `m8_disturb.yaml`의 `disturb_kpi` 추출 버그를 수정하고, 기본 disturbance magnitude를 `60N x 0.10s`로 낮춰 OFF/ON 차이가 보이도록 다시 맞춘다.
- M8에서 `world/global` force와 controller가 쓰는 `g1_imu_link` control frame이 계속 어긋나는 문제가 보여, disturbance 입력에도 `frame_mode=g1_imu_link`를 추가했다. 이제 설정 `force_xyz`는 control-frame 기준으로 해석하고, `world.py`에서 torso local wrench로 remap한 뒤 `configured_*`와 `applied_*`를 로그에 함께 남긴다. 이 수정은 M8 외란 입력 경로만 바꾸므로 M5/M7 standing baseline 재검증까지는 요구하지 않는다.
- `M8` 실행 오케스트레이션도 단순화한다. `ops/run_m8_pair.sh`를 추가해 같은 `RUN_ID` 아래에서 `balance_off -> tmux session 종료 -> balance_on -> 종료`를 순차 실행하고, 로그는 `logs/sim2real/m8/<run_id>/balance_off|balance_on/`에 바로 저장되게 정리한다.
- `M8` pair wrapper는 `tmuxp load -d -y` detached 실행으로 고친다. 기존 attach 모드에서는 첫 세션이 끝난 뒤에야 스크립트가 복귀해 `has-session` 대기 로직이 오작동했고, 실제로는 run이 끝났는데도 `did not start within 15s`처럼 보였다. disturbance도 `60N x 0.10s`는 너무 약해 `balance_off` 기준 `peak_abs_tilt_p≈0.028` 수준에 그쳐 `90N x 0.10s`로 한 단계 다시 올린다.
- `90N x 0.10s`에서도 `balance_off`와 `balance_on`이 모두 no-fall이면, M8 목표는 이제 `OFF만 fall / ON은 no-fall`인 경계값을 찾는 것으로 본다. force만 올리는 단일 변수 탐색 원칙에 따라 다음 기본 disturbance는 `120N x 0.10s`로 조정한다.
- `120N x 0.10s`, `110N x 0.10s`, `100N x 0.10s` 탐색과 `is_global=false` 전환만으로는 여전히 `roll` 우세 응답이 남았다. 그래서 M8 외란 입력을 observer와 같은 control frame으로 맞추기 위해 `disturbance.frame_mode=g1_imu_link`를 도입했고, `configured_force_xyz`와 실제 torso local `applied_force_xyz`를 로그에 함께 남기도록 정리했다.
- 위 frame alignment 이후에는 `balance_off`는 no-fall인데 `balance_on`만 과민하게 반응해 fall하는 패턴이 확인됐다. 이에 따라 `stand_pd_balance_on.yaml`에서 `tilt_qref_bias_abs_max: 0.15 -> 0.08`, `tilt_kp_pitch: 20 -> 12`, `stand_kp: 60 -> 50`, `stand_kd: 4 -> 3.5`로 완화했고, 현재 M8은 `OFF/ON` 모두 no-fall이지만 `ON`의 roll 응답이 여전히 큰 상태라 disturbance 대응용 balance tuning을 계속 진행한다.
- M8 비교군 정의도 다시 바로잡는다. 이제 `stand_pd_balance_base.yaml` 1개를 공통 baseline으로 두고, `ops/run_m8_pair.sh` + `controller.launch.py`가 `enable_tilt_feedback_override=true/false`만 런타임에서 넘긴다. 즉 이후 M8의 OFF/ON 차이는 정말 balance feedback 유무만 의미하고, 매 run마다 별도 YAML을 생성하지 않는다.
- 위 공정 비교에서도 `balance_on`이 `balance_off`보다 더 크게 흔들리는 것이 확인돼, 현재 M8은 구조/좌표계 문제가 아니라 disturbance-response gain tuning 단계로 본다. 이에 따라 공통 base에서 `tilt_qref_bias_abs_max: 0.08 -> 0.04`, `tilt_kp_roll/kd_roll: 4.0/0.8 -> 2.0/0.4`, `tilt_kp_pitch/kd_pitch: 12.0/2.0 -> 8.0/1.2`로 다시 보수화해 ON feedback의 과민 응답을 줄이는 방향으로 진행한다.
- 이후 `113N`에서 `OFF fall / ON survive`가 한 번 나오고, `115N`에서는 `ON`도 더 늦게 넘어지며 peak tilt를 줄이는 패턴이 확인됐다. 다만 반복 3회 기준 재현성이 아직 부족해, 현재는 roll gain은 유지한 채 `tilt_qref_bias_abs_max: 0.04 -> 0.05`, `tilt_kp_pitch/kd_pitch: 8.0/1.2 -> 9.0/1.4`로 `pitch authority`만 소폭 보강해 113N 재현성과 115N margin을 조금 더 확보하는 쪽으로 진행한다.
- 최근 `115N` 런에서는 `balance_on`이 `balance_off`보다 훨씬 늦게 넘어지고 peak tilt도 더 작았지만, `loop_after_disturb`에서 `u_pitch=0.050` 상한에 계속 걸리는 패턴이 확인됐다. 현재 bottleneck은 gain 부족보다 `pitch qref bias cap`에 가까우므로, `tilt_qref_bias_abs_max: 0.05 -> 0.06`으로 상한만 소폭 열어 `115N` margin을 더 확보하는 쪽으로 조정한다.
- `tilt_qref_bias_abs_max`를 `0.07`, `0.08`까지 올리면 오히려 `balance_on`이 `balance_off`보다 빨리 무너지거나 비슷한 시점에 fall하는 패턴이 다시 나타났다. 현재 M8 기준점은 `qref_bias_max=0.05`로 두고, 다음 탐색축은 총량이 아니라 `pitch allocation`으로 본다.
- 현재는 같은 `u_pitch` 총량을 힙보다 발목에 더 보내는 방향으로 조정한다. `tilt_weight_pitch_hip: 0.80 -> 0.60`, `tilt_weight_pitch_ankle: 0.70 -> 0.90`, `tilt_weight_pitch_torso: 0.05 -> 0.00`로 바꿔, disturbance 대응을 상체/힙 과반응보다 발목 복원 중심으로 다시 맞춘다.
- 위 `pitch allocation` 변경은 `20260314-170415` 런에서 악화로 확인됐다. 기존 `20260314-165912` 기준(hip=0.80, ankle=0.70, torso=0.05)에서는 `balance_on`이 `balance_off`보다 훨씬 오래 버티고 peak tilt도 크게 줄였지만, `hip↓/ankle↑` 변경 후에는 `balance_on`도 거의 같은 시점에 fall하고 `roll/pitch` KPI도 더 나빠졌다. 따라서 M8 기준점은 기존 분배로 되돌린다.
- 현재 M8 기준점은 `qref_bias_max=0.05`, `tilt_kd_pitch=1.4`, 기존 pitch 분배(hip=0.80, ankle=0.70, torso=0.05)다. 현재 우선순위는 `115N` margin 확장보다 `113N` repeatability 확보이며, 같은 설정에서 `balance_off`는 반복적으로 fall하고 `balance_on`은 반복적으로 survive하는지 먼저 확인한다.
- 최근 `112N` 반복 런에서는 `balance_on`이 매번 `balance_off`보다 훨씬 오래 버티고 `peak_abs_tilt_p_after_disturb`도 더 작았지만, `balance_on`의 마지막 병목은 여전히 sagittal/pitch margin으로 보인다. 다음 튜닝 축은 `pitch authority` 자체를 더 키우는 것이 아니라, `pitch P/D balance`를 바꿔 `bang-bang` 반응을 줄이는 쪽이다. 기준점은 `qref_bias_max=0.05`, 기존 pitch 분배 유지한 채 `tilt_kp_pitch: 9.0 -> 8.0`, `tilt_kd_pitch: 1.4 -> 1.6`으로 조정한다.
- GUI 관찰과 최근 `112N` 로그를 같이 보면, 첫 외란은 주로 옆으로 시작하지만 실제 붕괴는 그 복원 과정에서 원을 그리듯 자세가 전이되다가 앞쪽으로 무너지는 `secondary pitch collapse`에 가깝다. 따라서 다음 튜닝 축은 roll gain 추가가 아니라, 발목이 복원 과정의 속도성 출렁임을 더 잘 죽이게 하는 방향으로 둔다. 현재 M8 baseline에서 `stand_kd_scale_ankle: 1.5 -> 1.8`로만 소폭 올려 이 2차 앞뒤 붕괴가 줄어드는지 본다.
- 최근 `112N` 반복 비교(`20260314-174050/174239/174406` vs `20260314-175022/175304/175444`)에서 `stand_kd_scale_ankle: 1.8` 증가는 `balance_on`의 roll 응답과 생존 횟수를 개선했지만, `peak_abs_tilt_p_after_disturb`는 거의 줄지 않았고 `loop_after_disturb`에서 `u_pitch=0.050` 상한에 계속 걸렸다. 다음 탐색축은 발목 damping이 아니라 `pitch qref bias cap`이며, `tilt_qref_bias_abs_max: 0.05 -> 0.06`으로 한 칸만 다시 연다.
- 위 `0.06` 재실험(`20260314-180142`, `20260314-180311`)에서는 `balance_on`이 오히려 `balance_off`보다 더 빨리 무너지고 `roll/pitch` KPI도 더 악화됐다. 해석은 `dynamic pitch cap` 부족이 아니라, nominal standing pose 자체가 sagittal 방향으로 약간 앞쪽에 있다는 쪽이다.
- 따라서 M8 기준 pose 운영은 `q_ref_final = stand_q_ref(baseline) + stand_q_ref_trim + qref_bias_dynamic` 구조로 분리한다. IsaacLab nominal pose는 baseline으로 유지하고, 튜닝은 `stand_q_ref_trim`만 건드린다.
- 1차 trim은 controller가 기존에 반복적으로 넣던 보정 방향과 맞춘다. `tilt_qref_bias_abs_max`는 `0.05`로 되돌리고, `stand_q_ref_trim`에서 `left/right_hip_pitch_joint`와 `left/right_ankle_pitch_joint`만 `+0.01rad` posterior-safe 방향으로 소폭 이동해 `secondary pitch collapse`가 줄어드는지 확인한다.
- 위 trim 적용 후 `20260314-181557`, `20260314-181756`에서는 `balance_off`가 둘 다 no-fall로 좋아졌지만 `balance_on`은 오히려 더 빨리 무너졌다. 해석은 trim 방향 자체는 맞지만, 같은 방향의 `qref_bias_dynamic`가 예전과 동일 cap(`0.05`)으로 또 들어가며 과보정이 생긴다는 것이다. 다음 단계는 trim은 유지하고 `tilt_qref_bias_abs_max: 0.05 -> 0.03`으로 낮춰 dynamic bias 상한만 줄이는 것이다.
- 위 `trim + qref_bias_max=0.03` 조합으로 `20260314-182202`를 다시 보면 `balance_off/on`이 둘 다 no-fall로 내려왔다. 해석은 tuning 방향은 맞지만 `112N`이 더 이상 경계 외란이 아니라는 뜻이므로, 다음 검증은 같은 controller 세팅을 유지한 채 disturbance magnitude만 `113N`으로 다시 올려 `OFF fall / ON no-fall` 경계를 재탐색한다.
- 이후 `113N` 2회(`20260314-182500`, `20260314-182632`)에서는 `balance_off/on`이 모두 fall했고, `112N` 2회(`20260314-182815`, `20260314-183002`)에서는 `balance_on`은 2/2 no-fall인데 `balance_off`가 1/2 no-fall로 남았다. 해석은 `qref_bias_max=0.03` 방향은 맞지만, 공통 `stand_q_ref_trim=+0.01`이 `OFF`까지 너무 강하게 살리고 있다는 것이다. 다음 단계는 disturbance를 더 올리는 것이 아니라 `stand_q_ref_trim`의 hip/ankle pitch를 `+0.005`로 절반만 남겨 `112N`에서 `OFF fall / ON no-fall` 분리를 더 선명하게 만드는 것이다.
- 위 `stand_q_ref_trim=+0.005`, `qref_bias_max=0.03` 조합으로 다시 측정한 결과, `112N` 3회(`20260314-183852`, `20260314-184019`, `20260314-184146`)에서는 `balance_on`이 3/3 no-fall이고 `balance_off`는 1/3 fall에 그쳤다. 반면 `113N` 3회(`20260314-184316`, `20260314-184442`, `20260314-184609`)에서는 `balance_off`가 3/3 fall, `balance_on`이 3/3 no-fall로 깔끔하게 분리됐다. 따라서 현재 M8 대표 disturbance 조건은 `113N x 0.10s sagittal torso impulse`로 고정한다.
- M9 최소 구현은 기존 M8 raw log를 깨지 않고 읽는 후처리 레이어로 둔다. `scripts/sim2real/extract_m8_kpi.py`가 `fall_event/disturb_kpi/reason_count/sync_markers/start`를 읽어 `logs/sim2real/m9/<run_id>/balance_off_kpi.json`, `balance_on_kpi.json`, `comparison.json`, `summary.md`, 누적 `logs/sim2real/m9/index.csv`를 생성한다.
- 실행 레이어는 3단계로 분리한다. `ops/run_m8_pair.sh`는 raw 실험만 담당하고 `logs/sim2real/m8/<run_id>/`만 만든다. `ops/run_m9_kpi.sh <run_id>`는 이미 있는 M8 raw를 읽어 `m9/<run_id>/`만 생성한다. 편의용으로는 `ops/run_m8_and_m9.sh`를 두어, 실험 후 바로 M9 후처리까지 한 번에 묶어 돌릴 수 있게 한다.
- `ops/run_m8_and_m9.sh`는 내부에서 하위 스크립트를 직접 실행하지 않고 `bash`로 명시 호출한다. 이렇게 해야 `chmod +x` 유무와 무관하게 wrapper가 항상 동작하고, 실제 사용자 실행 방식(`bash ops/...`)과도 일관된다.
- M9 산출물 중 `kpi.json` / `comparison.json`은 기계용으로 유지하고, `summary.md`만 사람용으로 더 짧게 줄인다. summary는 `verdict / outcome / key KPI / 핵심 설정 2개`만 남기고 dict/raw 경로 같은 저수준 정보는 제거한다.
- M9 `summary.md`는 최종적으로 표 형식으로 고정한다. Header 아래에 `run_id / verdict / disturbance`를 두고, `Outcome`, `Key KPI`, `Config`는 모두 markdown table로 출력한다. metric 이름은 JSON 키와 맞추고, 설정은 `tilt_qref_bias_abs_max`, `hip_pitch_joint_trim`, `ankle_pitch_joint_trim`만 노출한다.
- README / overview / one-pager / master plan을 M9 완료 상태로 동기화한다. 이제 현재 게이트는 `포트폴리오 정리`이고, M9는 `M8 raw -> M9 summary` 자동화까지 끝난 상태로 본다.
- 공식 milestone 체계에서 `Portfolio Packaging`을 제거하고, 기존 `M11~M18`을 `M10~M17`로 재번호했다. `MASTER_PLAN / README / overview / ONE_PAGER / KIST insert layout`을 새 번호 체계로 다시 동기화한다.
- 이후 확장 milestone 흐름도도 실무 순서 기준으로 다시 정리한다. `Experiment Infrastructure`를 `Sim-to-Real Hardening`보다 앞에 두고, `MASTER_PLAN`과 KIST용 Slide 5 로드맵을 같은 순서로 맞춘다.
- 위 gain 보수화 후 `100N x 0.10s`에서는 `balance_off/on` 모두 no-fall이었고, `balance_on`이 `balance_off`보다 `peak_abs_tilt_r`를 더 작게 유지했다. 다음 단계는 같은 설정에서 force만 `105N`으로 올려 `OFF fall / ON survive` 경계값을 찾는 threshold search다.
- M8 영상 캡처 목적도 반영한다. `m8_disturb`는 더 이상 `fall 시 즉시 종료`가 아니라 `stop_on_fall_event=false`로 두고, `FALL_EVENT`는 로그만 남긴 채 pane 3의 고정 관측창이 끝날 때까지 계속 재생한다. 외란 시작은 단순 `sim elapsed`가 아니라 `CONTROL_ACTIVE marker + 3.0초` 기준으로 맞춘다. tmux pane 3가 `[SYNC] CONTROL_ACTIVE`를 잡으면 marker 파일을 만들고, `world.py`가 이를 감지해 `DISTURBANCE_START`를 동일한 제어 기준 시점에 맞춰 split-screen 비교가 쉬운 fixed-window 영상으로 남긴다. `start_elapsed_sec=5.0`은 marker 실패 시 fallback으로만 유지한다.
- `logs/sim2real/` 운영 규칙도 재정리한다. 기존 run-id 기반 exploratory 로그는 `_legacy/` 아래로 묶어 보존하고, 앞으로 생성되는 로그는 `logs/sim2real/<milestone>/<run_id>/...` milestone-first 구조로 고정한다.
- 포트폴리오 공개용 대표 증빙도 같은 규칙으로 정리한다. `M5`, `M7` 대표 run은 `_legacy`에서 `logs/sim2real/m5/20260314-121949_m5_stand_sanity_qrefv7/`, `logs/sim2real/m7/20260314-133954_m7_stand_safecheck/`로 복사해 milestone-first 경로로 맞췄고, `README/overview/ONE_PAGER`는 이 새 경로와 최신 `M9` 대표 run(`logs/sim2real/m9/20260315-000113/`)을 보도록 갱신했다. `.gitignore`는 여전히 `logs/` 전체를 막되, 위 대표 파일들만 예외로 공개한다.
- 공개용 문서 세트도 다시 동기화한다. `README`는 포트폴리오 랜딩 페이지 구조를 유지한 채 `final demo video`, `system_architecture`, `milestone_roadview` 자산을 직접 연결하고, `overview / ONE_PAGER / ROADMAP / AGENTS / KIST insert / general insert / VIDEO_CAPTURE_GUIDE`도 같은 milestone 체계와 milestone-first 증빙 경로로 맞춘다. 로컬 최종본 mp4는 `reports/sim2real/video/RB_Humanoid_Control.mp4`로 정리했다.
- README 최종 마감에서는 구조도와 로드맵을 링크가 아니라 inline 이미지로 직접 노출하고, 시연영상은 mp4 자체 embed 대신 대표 still 이미지를 클릭하면 final demo asset으로 이동하는 방식으로 정리한다.
- GitHub README 공개본에서는 repo mp4 대신 `user-attachments` 영상 asset(`https://github.com/user-attachments/assets/4e70156b-aca6-4c3a-859f-7526fa2f511e`)을 최종 시연영상으로 사용한다. repo 내부 mp4는 공개 README 링크의 canonical source로 쓰지 않는다.

### 2026-03-26 추가 정리 — IMU startup bias 제거 / M10 진입 준비
- `imu_zero_on_start`와 startup bias 캡처/차감 경로를 제거했다. 현재 standing baseline은 `imu_frame_mode=g1_imu_link` frame compensation만 사용하고, startup zeroing에는 더 이상 의존하지 않는다.
- `rb_controller/msg/EstimatedState.msg`에서 `bias_roll_rad`, `bias_pitch_rad`를 제거했다. estimator/controller/debug logger도 같은 방향으로 정리해 `/rb/estimated_state` 계약을 더 단순하게 맞췄다.
- `controller_tilt_observer.*`, `estimator_node.cpp`, `controller_node.cpp`, `controller.launch.py`, standing 시나리오 YAML에서 bias 관련 파라미터/로그/launch fallback을 제거했다. 즉 현재 구조는 `joint + raw IMU 해석 + frame remap -> tilt/rate`만 남긴 상태다.
- 추가로 `imu_frame_mode`, `tilt_axis_mode`, legacy alias 인터페이스도 제거했다. 현재 IMU frame correction은 estimator 내부의 G1 고정 보정으로만 동작하고, `EstimatedState.msg`와 standing 시나리오 YAML에서도 관련 옵션을 더 이상 노출하지 않는다.
- `colcon build --packages-select rb_controller` 재빌드 통과. 메시지 타입 변경 후에도 estimator/controller/safety 경로가 다시 맞물리는 것까지 확인했다.
- no-bias sanity 확인도 했다. `logs/sim2real/m7/test_no_bias_20260326_093821/m7/` 기준으로 `CONTROL_ACTIVE` 시점 `tilt_r/p`가 0 근처로 들어왔고, safety-on 60초 관측창에서 `NO_FALL_EVENT`, `NO_SAFETY_REASON`를 다시 확인했다.
- M8/M9 회귀도 다시 확인했다. `logs/sim2real/m8/test_no_bias_20260326_093821/`, `logs/sim2real/m9/test_no_bias_20260326_093821/summary.md` 기준 현재 representative run은 `115N x 0.10s` disturbance에서 `balance_off/on` 모두 no-fall이고, M9 summary/KPI 생성까지 정상 동작한다.
- M9 후처리 스크립트 안정화도 같이 했다. `ops/run_m9_kpi.sh`는 모듈 실행(`python3 -m ...`)으로 바꿔 import 경로 문제를 없앴고, `scripts/sim2real/kpi/writers.py`는 optional config 값이 `None`일 때도 `summary.md`를 정상 생성하도록 보강했다.
- 포트폴리오용 기존 M8/M9 결론(`같은 disturbance에서 OFF fall / ON survive`가 관측된 대표 baseline이 존재한다)은 유지한다. 이번 bias 제거 런은 성능 결론을 다시 덮어쓰는 실험이 아니라, estimator contract 단순화 이후에도 standing/disturbance/KPI 파이프라인이 여전히 정상 동작하는지 보는 회귀 확인으로 취급한다.
- 다음 초점은 M10이다. 현재 estimator 분리 자체는 끝났고, 다음 단계는 `EstimatedState`를 더 명확한 제어용 상태 계약으로 다듬고, 그 상태를 controller/safety가 더 일관되게 쓰게 만드는 쪽이다.
