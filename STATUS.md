# STATUS — RB_Humanoid_Control

## 1. 현재 프로젝트 한 줄 요약
휴머노이드 제어 엔지니어 포지션 지원을 위해, 실기체 없이 Isaac Sim + ROS2 기반 휴머노이드 제어 시스템을 직접 구성하고 증빙하는 프로젝트.

## 2. 현재 고정된 기준
### active 실행 경로
- `rb_estimation -> rb_standing_controller -> rb_hardware_interface -> rb_safety -> Isaac`
- 즉 active runtime은 native `ros2_control` standing stack 기준이다.

### 유지
- 로봇 모델은 G1 유지
- Gym/IsaacLab Stage1 코드는 baseline/archive로 유지
- 실행 진입점은 루트 `main.py` 유지
- 메인 경로는 `원본 G1 asset/cfg direct spawn + standalone World.step()`

### legacy 처리
- `rb_controller`는 active runtime에서 제외됐다.
- 현재는 `COLCON_IGNORE`가 걸린 legacy/archive 패키지로만 유지한다.
- active tmux/profile/wrapper는 더 이상 `rb_controller`에 의존하지 않는다.

### source of truth
- joint ordering: [joint_order_g1.yaml](/home/leemou/Projects/RB_Humanoid_Control/ros2_ws/src/rb_bringup/config/joint_order_g1.yaml)
- standing seed/reference: [stand_qref_g1_seed.yaml](/home/leemou/Projects/RB_Humanoid_Control/ros2_ws/src/rb_bringup/config/stand_qref_g1_seed.yaml)
- 공통 standing controller baseline: [standing_controller_baseline.yaml](/home/leemou/Projects/RB_Humanoid_Control/ros2_ws/src/rb_bringup/config/standing_controller_baseline.yaml)
- milestone stack profile:
  - [m5_stack_relaxed.yaml](/home/leemou/Projects/RB_Humanoid_Control/ros2_ws/src/rb_bringup/config/m5_stack_relaxed.yaml)
  - [m7_stack_safety_on.yaml](/home/leemou/Projects/RB_Humanoid_Control/ros2_ws/src/rb_bringup/config/m7_stack_safety_on.yaml)
  - [m8_stack_disturb.yaml](/home/leemou/Projects/RB_Humanoid_Control/ros2_ws/src/rb_bringup/config/m8_stack_disturb.yaml)

## 3. 현재 상태
### 구조
- `rb_interfaces`, `rb_estimation`, `rb_safety`, `rb_standing_controller`, `rb_hardware_interface`, `rb_bringup` 기준으로 패키지 경계를 정리했다.
- `rb_standing_controller`는 active custom controller plugin이고, `rb_hardware_interface`는 active hardware plugin이다.
- startup sequencing 때문에 초기에 찍히던 `TIMEOUT`은 `rb_safety`를 "첫 valid command 이후에 arm"하도록 바꿔 정리했다.

### standing / revalidation
- native standing path에서 실제로 선다.
- legacy `m7` safecheck도 다시 서는 것을 확인했고, native path도 같은 기준에서 standing을 재확인했다.
- 핵심 수정은 IMU publisher를 뒤집는 게 아니라, estimator의 tilt observer가 `g1_imu_link -> control frame` 기준으로 tilt/rate를 해석하도록 유지한 것이다.
- xacro/source-of-truth 정합성도 다시 맞췄다.
  - `rb_safety.joint_limits.*`는 xacro와 일치
  - `rb_standing_controller.limit_avoid_lower`의 ankle pitch lower도 xacro 기준으로 수정
  - `stand_q_ref`와 runtime joint ordering도 다시 맞춤

### milestone 운영
- `M5/M7/M8`은 공통 standing baseline 하나를 사용한다.
- milestone마다 달라지는 것은 controller gain이 아니라 stack/safety profile과 실험 스위치다.
- `M8/M9` 대표 증빙은 기존 결과를 유지하고, 현재 focus는 `M10` 선행 정리와 이후 `M11+` 확장 준비다.

### 현재 보류 / 후속 이슈
- `rb_controller`는 아직 문서/legacy tmux 참조가 남아 있어 즉시 삭제하지 않고 archive로 보관한다.
- `M8`의 `enable_tilt_feedback` ON/OFF override는 현재 launch override 정합성을 다시 봐야 한다.
  - 이건 `M11`의 metrics/tuning 단계에서 함께 정리할 예정이다.

## 4. 상세 작업 로그 / 변경 이력
아래부터는 날짜별 실험 메모와 구조 변경 이력을 누적해 둔 구간이다.
현재 active truth는 위 `2. 현재 고정된 기준`, `3. 현재 상태`를 우선 기준으로 본다.


### 최근 조사 결과
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
- `joint sign`을 육안 대신 숫자로 확인하기 위한 최소 audit 경로 추가: `joint_sign_audit.yaml` + `ops/tmuxp/legacy/joint_sign_audit.yaml` + `scripts/sim2real/joint_state_delta.py`. 단일 joint에 작은 `step_effort`를 주고 `js_before/js_after`에서 `delta` 부호를 계산하도록 정리
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
- `M4` safety 증빙도 보강한다. `velocity.yaml` + `ops/tmuxp/legacy/m4_velocity.yaml`을 추가해 팔 관절 하나를 빠르게 움직이는 전용 시나리오로 `reason=VELOCITY_LIMIT`를 재현하고, `reasons.txt`와 `loop_tail.txt`를 기존 M4 reason과 같은 방식으로 수집한다.
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
- M8 비교군 정의도 다시 native 기준으로 바로잡았다. 이제 `ops/run_m8_pair.sh`는 공통 controller baseline `rb_bringup/config/standing_controller_baseline.yaml`과 M8 stack profile `rb_bringup/config/m8_stack_disturb.yaml`을 그대로 쓰고, launch override로 `enable_tilt_feedback`만 `false/true` 바꾼다. 즉 이후 M8의 OFF/ON 차이는 정말 native standing plugin의 `enable_tilt_feedback` 유무만 의미한다.
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

### 2026-03-26 추가 정리 — M10 1차 완료
- `rb_controller/msg/EstimatedState.msg`에 `joint_state_stamp`, `imu_stamp`, `joint_state_valid`, `imu_valid`를 추가했다. 이제 `/rb/estimated_state`는 값만 담는 메시지가 아니라, source freshness와 validity도 같이 담는 제어 계약이 됐다.
- `rb_controller`의 build 인터페이스도 같이 정리했다. `package.xml`, `CMakeLists.txt`에 `builtin_interfaces` 의존성을 추가해 새 message 필드가 정상적으로 생성되도록 맞췄다.
- `estimator_node.cpp`는 source stamp cache와 `input_stale_timeout_sec` 기반 stale 판정을 추가해, joint/imu가 오래된 경우 `joint_state_valid`, `imu_valid`를 false로 publish하도록 바꿨다.
- `controller_node.cpp`는 valid joint/imu가 들어오기 전까지 실제 standing control을 시작하지 않도록 정리했다. 지금 `[SYNC] CONTROL_ACTIVE`는 사실상 "estimated_state 준비 완료" marker 역할을 한다.
- `safety_node.cpp`도 same contract를 쓰게 맞췄다. 현재 safety는 `imu_valid=false`인 상태를 tilt safety 판단에 쓰지 않는다.
- M10 1차 스모크도 통과했다. build 후 `ros2 interface show rb_controller/msg/EstimatedState`에서 새 필드가 보이는 것, `m7_stand_safecheck`에서 `CONTROL_ACTIVE / NO_SAFETY_REASON / NO_FALL_EVENT`가 유지되는 것을 확인했다.
- 복습 문서도 이번 구조에 맞춰 정리했다. `docs/review/m10/00_index.md`, `01_contract.md`, `02_file_changes.md`, `03_smoke_check.md`를 기준으로 `msg -> estimator -> controller -> safety` 순서로 복습할 수 있게 했다.
- 현재 판단: M10 1차 범위는 완료로 본다. 다음은 `ros2_control` 구조 전환을 다루는 M10.5다.

### 2026-03-26 추가 정리 — M10.5 Step 1 시작 (ros2_control 골격 추가)
- `rb_bringup`은 더 이상 단순 config 폴더가 아니다. `package.xml`, `CMakeLists.txt`를 추가해 실제 ROS 패키지로 승격했고, `launch / config / urdf`를 install space 기준으로 배포할 수 있게 정리했다.
- `ros2_control` 최소 bringup을 추가했다. `rb_bringup/launch/ros2_control.launch.py`는 control-only xacro를 읽어 `controller_manager(ros2_control_node)`, `robot_state_publisher`, 필요 시 `joint_state_broadcaster spawner`를 띄운다. `start_joint_state_broadcaster` 스위치도 넣어 broadcaster 없이 manager/hardware만 따로 점검할 수 있게 했다.
- `rb_bringup/config/standing_controller_baseline.yaml`도 추가했다. Step 1에서는 `joint_state_broadcaster`만 선언했지만, Step 2 시작과 함께 write path 점검용 표준 controller `rb_effort_forward_controller`를 추가했다. 이후에는 `rb_standing_controller` custom plugin까지 추가해 native controller 경로도 같이 검증할 수 있게 했다.
- `rb_bringup/urdf/g1_ros2_control.xacro`도 만들었다. 처음에는 dummy tree placeholder였지만, 이후 Isaac Script Editor에서 `g1_stage.usd`를 연 상태로 joint dump를 뽑아 실제 구조 기반으로 다시 썼다. 현재 xacro는 full URDF는 아니지만 `pelvis` root 기준 `43`개 joint tree(`37` movable + `6` fixed), parent-child, joint type, axis, limit을 반영한 control-only description이다. visual/collision mesh는 넣지 않고, `bridge_enabled`, `joint_state_topic`, `command_topic`을 launch arg로 주입할 수 있게 했다.
- 새 패키지 `rb_hardware_interface`를 추가했다. 현재 구현체 이름은 `RBHardwareSystem`이며, `hardware_interface::SystemInterface`를 상속하는 skeleton이다. 현재는 `effort` command와 `position/velocity/effort` state interface를 export하는 최소 드라이버 골격을 구현했다. Step 2부터는 launch에서 `bridge_enabled=true`를 주면 `/rb/joint_states`를 읽고 `command_topic`으로 effort 명령을 쓰는 초기 topic bridge 경로도 함께 올라간다.
- 중요한 점: 이번 Step 1은 direct apply 경로를 아직 제거하지 않는다. 현재 Isaac graph의 `/rb/command_safe -> articulation apply` 경로는 그대로 두고, 그 옆에 `ros2_control` 골격을 새로 세운 단계다.
- build/launch 검증도 했다. `colcon build --packages-select rb_bringup rb_hardware_interface rb_controller` 통과, xacro generate 통과, `ros2 launch rb_bringup ros2_control.launch.py start_joint_state_broadcaster:=false`에서 `controller_manager`가 `G1ControlOnlySystem` hardware를 `load -> init -> configure -> activate`하는 것과 `/controller_manager/*` 서비스 목록, `ros2 control list_hardware_interfaces` 기준 37개 effort/state interface 노출을 확인했다. 구조를 실제 joint tree로 바꾼 뒤에도 skeleton launch가 그대로 살아 있다는 점을 재확인했다.
- 이후 원인 분리도 했다. `list_controller_types`는 정상이었지만 `load_controller`가 멈춰 broadcaster 전용 문제가 아니라 controller loading 경로 전체 문제로 좁혔다. 다시 테스트한 결과 `controller_manager_use_sim_time=false`에서는 `joint_state_broadcaster`가 `load -> configure -> activate`까지 즉시 성공했고, `controller_manager_use_sim_time=true`에서는 `/clock`이 없을 때 timeout, 가짜 `/clock`을 넣으면 빠르게 실패하는 쪽으로 증상이 바뀌었다. 즉 “Isaac이 안 떠서 /clock이 없어서만 멈춘다”가 아니라, 현재 Humble 조합에서 controller_manager의 sim time 경로가 불안정한 상태로 본다.
- 그래서 launch를 정리했다. `use_sim_time`은 주변 sim 노드용으로 유지하되, `controller_manager_use_sim_time`을 별도 arg로 분리하고 기본값을 `false`로 뒀다. 현재 Step 1 기본 bringup은 이 설정에서 정상 통과한다.
- Step 2 초기 연결도 시작했다. `bridge_enabled:=true`, `command_topic:=/rb/command_raw`, `start_effort_forward_controller:=true`로 launch했을 때 `RBHardwareSystem configured: subscribed=/rb/joint_states published=/rb/command_raw`와 `Loaded/Configured and activated rb_effort_forward_controller`까지 확인했다. 즉 ros2_control skeleton이 이제 기존 Isaac direct-apply graph의 `/rb/command_raw -> articulation` 경로 앞단에 붙을 준비가 된 상태다.
- `/rb_effort_forward_controller/commands`에 effort 배열을 보냈을 때 `/rb/command_raw --once`에서 같은 effort 배열이 실제로 찍히는 것도 확인했다. 즉 ros2_control write path는 확인됐고, 다음 확인 대상은 이후 `m3_command` phase의 `/rb/command_raw -> articulation` apply 구간이다.
- standing 재현용 active 세션은 이제 [m5_stand.yaml](/home/leemou/Projects/RB_Humanoid_Control/ops/tmuxp/m5_stand.yaml)와 [m7_stand_safecheck.yaml](/home/leemou/Projects/RB_Humanoid_Control/ops/tmuxp/m7_stand_safecheck.yaml)다. 둘 다 `m5_stand` scene 위에 native plugin path를 띄우고, relaxed/safety-on만 나눠서 본다.
- bridge-only standing smoke는 실제로 통과했다. `m5_stand` standing scene에서 effort test 후 **왼팔이 뒤로 움직이는 것**을 확인했고, 최소 `ros2_control -> /rb/command_safe -> Isaac apply` 경로는 검증됐다.
- 다만 여기까지는 어디까지나 **bridge-only smoke**다. 기존 `rb_controller -> rb_safety` 제어 출력이 ros2_control을 거쳐 apply되도록 완전히 옮긴 것은 아직 아니다.
- 그래서 `rb_controller/src/command_bridge_node.cpp`를 추가했다. 이 노드는 `rb_safety`가 내는 `sensor_msgs/msg/JointState` effort 명령을 받아 `std_msgs/msg/Float64MultiArray`로 바꿔 `/rb_effort_forward_controller/commands`로 넘긴다.
- `rb_controller/launch/controller.launch.py`에도 `enable_command_bridge`, `command_bridge_input_topic`, `command_bridge_output_topic`, `ros2_control_controllers_file` 인자를 추가했다. 이 모드에서는 `rb_safety` 출력이 `/rb/command_safe_source` 같은 중간 토픽으로 나가고, `rb_command_bridge`가 그 값을 ros2_control forward controller 입력으로 넘긴다.
- 위 active stand 세션들은 `m5_stand + ros2_control + native_stack` 조합으로 통일했다. 즉 이제 stand smoke는 `rb_estimator -> rb_standing_controller -> RBHardwareSystem -> rb_safety -> Isaac` 경로만 본다.
- standing smoke tmuxp는 이제 full-stack path 확인만 남긴다. 즉 `rb_controller -> rb_safety -> rb_command_bridge -> rb_effort_forward_controller -> /rb/command_safe -> Isaac` 경로가 실제로 살아 있는지만 로그로 확인한다.
- 현재 launch에서 보이는 `FIFO RT scheduling`, `getifaddrs`, root inertia 관련 경고는 Step 1 실패가 아니라 환경/description 제약이다.
- 복습 문서도 같이 추가했다. `docs/review/m10_5/00_index.md`, `01_contract.md`, `02_file_changes.md`, `03_smoke_check.md`를 기준으로 이번 단계에서 무엇을 만들었고 아직 무엇을 안 했는지 바로 따라갈 수 있게 정리했다.
- 이후 full-stack smoke에서도 `command_safe_source -> effort_controller -> command_safe` 세 hop이 실제로 살아 있는 것을 로그로 확인했다. 즉 현재 repo는 `custom -> adapter -> ros2_control -> Isaac` 전환형 구조까지는 검증된 상태다.
- 다만 이건 최종 목표 구조는 아니다. 지금 남아 있는 active path에는 아직 `rb_command_bridge`, `rb_effort_forward_controller`가 있고, standing 제어 수학 자체는 `controller_node.cpp` 일반 ROS 노드에 남아 있다.
- 따라서 현재 판정은 이렇게 잡는다.
  - 완료: `ros2_control` 전환형 구조 검증
  - 완료: `rb_standing_controller` native ros2_control controller plugin + stand_pd 핵심 계산 이식
  - 미완료: active path 교체
- 이 차이를 헷갈리지 않도록 `docs/review/m10_5/04_ros2_control_study.md`를 추가해, 현재 구조와 최종 목표 구조를 초보자 기준으로 다시 설명했다.

### 2026-03-27 추가 정리 — native standing controller 추가
- 새 패키지 `rb_standing_controller`를 추가했다. 이 패키지는 `controller_interface::ControllerInterface` 기반 custom controller plugin 자리다.
- 현재 구현체 이름은 `RBStandingController`다. 이제는 zero-effort skeleton이 아니라, `joints`, `command_interface_name`, `state_interface_names`를 기준으로 `37`개 effort/state interface를 claim하고 stand_pd 핵심 계산을 직접 수행한다.
- `rb_bringup/config/standing_controller_baseline.yaml`에 `rb_standing_controller` 항목을 추가했다. 현재는 기존 `rb_effort_forward_controller`와 별도로, `start_standing_controller:=true`일 때만 custom controller를 스폰한다.
- `rb_bringup/launch/ros2_control.launch.py`에도 `start_standing_controller` 인자를 추가했다. 현재는 `rb_effort_forward_controller`와 같은 effort command interface를 claim하므로 둘을 동시에 켜지 않는다.
- build 검증도 했다. `colcon build --packages-select rb_standing_controller rb_bringup rb_hardware_interface rb_controller` 통과.
- bringup 검증도 했다. `ROS_LOG_DIR=/tmp/ros_logs ros2 launch rb_bringup ros2_control.launch.py start_standing_controller:=true start_effort_forward_controller:=false start_joint_state_broadcaster:=true bridge_enabled:=false`에서 아래가 실제로 확인됐다.
  - `Loaded rb_standing_controller`
  - `rb_standing_controller configured: joints=37 estimated_state_topic=/rb/estimated_state stand_kp=6.00 stand_kd=8.00 limit=1.40`
  - `rb_standing_controller activate successful: native stand_pd plugin is now driving effort interfaces`
  - `stand_q_ref size mismatch: ref=1 joint_count=37. fallback to current pose=enabled`
  - `native stand_pd reference ready: source=current_pose count=37`
  - `Configured and activated rb_standing_controller`
- 현재 판정:
  - 완료: native custom controller plugin 생성 + build + load/configure/activate 검증
  - 완료: 기존 `controller_node.cpp`의 stand_pd 핵심 계산을 `RBStandingController` 안으로 이식
  - 미완료: active path에서 `rb_command_bridge`, `rb_effort_forward_controller`를 제거하는 것

### 2026-03-27 추가 정리 — native plugin active-path smoke
- `RBStandingController` 코드의 비자명한 경계에 한국어 주석을 더 보강했다. 특히 `EstimatedStateInput`, `TiltFeedbackCommand`, `on_configure()`의 parameter snapshot 의도, `update()`의 `effective_q_ref` 사용 이유를 바로 읽히게 적었다.
- native active-path smoke를 별도로 실행했다. 조합은 아래 셋이다.
  - `python main.py --phase m5_stand --steps 1200 --headless`
  - `ros2 launch rb_bringup ros2_control.launch.py bridge_enabled:=true command_topic:=/rb/command_raw start_effort_forward_controller:=false start_standing_controller:=true`
  - `ros2 launch rb_controller controller.launch.py params_file:=.../stand_pd_safecheck.yaml start_controller:=false enable_command_bridge:=false`
- 로그는 `logs/sim2real/m10_5/smoke_20260327_native_plugin/`에 남겼다.
- 실제 관찰 결과:
  - `ros2_control_native.log` 기준 `rb_standing_controller`는 `load -> configure -> activate` 됐다.
  - `command_raw_once.txt`에는 nonzero leg effort가 찍혔다. 즉 native plugin이 실제로 `RBHardwareSystem.write(/rb/command_raw)`까지 명령을 만들었다.
  - `command_safe_once.txt`는 전부 `0.0`이었다. 이건 native path 실패가 아니라, `controller_native.log`에서 보이듯 `rb_safety`가 `TILT/TIMEOUT` 이유로 `/rb/command_safe`를 zeroing한 결과다.
- 현재 권장 active path는 이렇게 읽는다.
  - `rb_estimator -> RBStandingController -> RBHardwareSystem.write(/rb/command_raw) -> rb_safety -> /rb/command_safe -> Isaac`
- 따라서 현재 판정은 이렇게 갱신한다.
  - 완료: legacy bridge path 검증
  - 완료: native plugin path 검증
  - 남은 것: `rb_safety`를 외부 supervisor로 유지할지, 일부를 plugin 내부로 옮길지에 대한 구조 선택

### 2026-03-27 추가 정리 — native plugin을 기본 active path로 전환
- 기본 launch도 이제 native plugin 기준으로 뒤집었다.
- `rb_bringup/launch/ros2_control.launch.py`는 기본값이 `start_standing_controller=true`, `bridge_enabled=true`다. 즉 ros2_control bringup을 그냥 띄우면 `rb_standing_controller`가 기본으로 올라간다.
- `rb_bringup/launch/native_stack.launch.py`를 새로 추가했다. 이 launch는 active native path에서 `rb_estimator + rb_safety`만 올리고, 기본 YAML도 `rb_bringup/config/m7_stack_safety_on.yaml`을 쓴다.
- `rb_controller/launch/controller.launch.py`는 legacy fallback launch로만 남긴다. 기본값이 `start_controller=false`, `enable_command_bridge=false`인 이유도 이제 "active path가 아니라 legacy replay/fallback 용도"이기 때문이다.
- `ops/tmuxp`도 active/native와 legacy를 분리했다. 현재 루트에는 `m1_sensor`, `m5_pose_audit`, `m5_stand`, `m7_stand_safecheck`, `m8_disturb`만 남기고, pre-native controller-node 전용 세션은 `ops/tmuxp/legacy/`로 이동했다.
- native smoke 기본값도 정리했다. `m7_stack_safety_on.yaml`은 safety-on 기준(`tilt_limit_roll/pitch_rad=0.6`), `m5_stack_relaxed.yaml`은 debug 기준(`tilt_limit_roll/pitch_rad=1.2`)으로 나눴고, active tmux는 controller/plugin 활성화 후 시점의 `/rb/command_raw`, `/rb/command_safe`를 다시 캡처하도록 바꿨다.
- native smoke 재검증(`20260327-131710`, `20260327-132027`)에서는 `rb_standing_controller`가 `/rb/command_raw`에 nonzero effort를 쓰고도 `rb_safety`가 `reason=TILT axis=ROLL`로 `/rb/command_safe`를 zeroing했다. GUI 관찰상 실제 붕괴는 전방(sagittal)인데 roll 채널이 커졌으므로, 원인을 native plugin이 아니라 `rb_estimation/controller_tilt_observer.cpp`의 고정 swap 매핑으로 판단했다.
- 이후 legacy와 native가 둘 다 다시 못 서는 것을 확인했고, 원인이 ros2_control이 아니라 estimator의 IMU frame compensation을 중간에 풀어버린 데 있다는 점을 확인했다. 최종적으로는 G1 IMU raw frame -> control frame 보정을 다시 유지했다. 즉 현재 `controller_tilt_observer.cpp`는 `tilt_roll <- raw_pitch`, `tilt_pitch <- raw_roll`, `roll_rate <- gyro.y`, `pitch_rate <- gyro.x`를 사용해 control-frame standard roll/pitch를 publish한다.
- 위 수정 후 `20260327-142224_m10_5_native_stack`에서는 같은 forward fall이 이제 `reason=TILT axis=PITCH`로 잡혔다. 즉 native path에서 남은 문제는 더 이상 tilt 축 해석이 아니라, standing controller baseline 자체가 아직 약한 쪽이라는 뜻이다.
- 그래서 native 기본 controller 설정도 legacy에서 safety-on으로 서 있던 `M7 safecheck` baseline으로 올렸다. `rb_standing_controller`는 이제 `stand_kp/kd=60/4`, `stand_effort_abs_max=18`, `stand_hold_current_on_start=false`, 검증된 `stand_q_ref`, 관절군별 gain scale, `tilt_apply_mode=qref_bias`, `stand_tilt_cut_enable=false`를 기본으로 사용한다. `rb_safety`도 같은 baseline에 맞춰 `effort_abs_max_default=18`, `tilt_limit_roll/pitch_rad=0.6`로 되돌렸다.
- 그 다음 `20260327-142746_m10_5_native_stack`에서는 robot이 뒤로 붕괴했다. 이 로그를 보면 pitch 축 해석은 계속 정상(`axis=PITCH pitch=-0.605`)이지만, 이번에는 native plugin의 `joints:` 순서와 `stand_q_ref` source-of-truth 순서가 달랐다. 즉 native controller가 검증된 `stand_q_ref` 값을 잘못된 관절에 대입하고 있었다.
- 그래서 `rb_bringup/config/standing_controller_baseline.yaml`의 `rb_standing_controller.joints`와 `rb_effort_forward_controller.joints`를 모두 `joint_order_g1.yaml` 순서로 통일했다. 현재 native plugin은 이제 `stand_qref_g1_seed.yaml`와 같은 순서 기준으로 reference/command를 해석한다.

### 2026-03-27 추가 정리 — native standing 재안정화 및 truth 재정의
- native path와 legacy path가 한동안 같이 무너지던 원인을 다시 추적한 결과, 핵심은 `rb_estimation/controller_tilt_observer.cpp`의 IMU 해석 계약을 중간에 잘못 건드린 것이었다. 최종적으로는 G1 IMU raw frame을 control frame 기준으로 해석하는 기존 계약을 유지했고, 그 결과 legacy `m7` safecheck와 native standing path가 다시 모두 섰다.
- 이 시점부터 source of truth는 legacy `rb_controller`가 아니라 native path로 고정했다. 현재 active runtime은 `rb_estimation -> rb_standing_controller -> rb_hardware_interface -> rb_safety -> Isaac`이고, `rb_controller`는 `COLCON_IGNORE`가 걸린 legacy/archive 패키지로만 유지한다.
- `rb_controller`를 즉시 삭제하지 않는 이유도 정리했다. active runtime은 이미 의존하지 않지만, `README`, `STATUS`, `reports`, `MASTER_PLAN`, legacy tmux/scenario 참조가 남아 있으므로 지금 단계에서는 archive 보관이 맞다.

### 2026-03-27 추가 정리 — 마일스톤 실행 경로/설정 정리
- active tmux 경로는 native 기준으로 정리했다. 루트 `ops/tmuxp/`에는 `m1_sensor`, `m5_pose_audit`, `m5_stand`, `m7_stand_safecheck`, `m8_disturb`만 남기고, legacy 세션은 `ops/tmuxp/legacy/`로 이동했다.
- 마일스톤별 실행 profile도 `ops/profiles/`로 분리했다. 현재 active profile은 `m5_stand.env`, `m7_stand_safecheck.env`, `m8_disturb.env`다.
- config 이름도 현재 역할이 바로 드러나게 다시 정리했다.
  - 공통 controller baseline: `rb_bringup/config/standing_controller_baseline.yaml`
  - M5 relaxed stack: `rb_bringup/config/m5_stack_relaxed.yaml`
  - M7 safety-on stack: `rb_bringup/config/m7_stack_safety_on.yaml`
  - M8 disturbance stack: `rb_bringup/config/m8_stack_disturb.yaml`
- 현재 운영 원칙도 고정했다. `M5/M7/M8`은 공통 controller baseline 하나를 쓰고, milestone마다 달라지는 것은 controller gain이 아니라 stack/safety profile과 실험 스위치다.

### 2026-03-27 추가 정리 — 공통 standing baseline 재정의
- 공통 standing baseline은 이제 `standing_controller_baseline.yaml` 하나로 통일했다. 이 파일은 M8 tuned 값을 기준으로 승격한 baseline이다.
- 현재 핵심값은 아래와 같다.
  - `stand_kp=50.0`
  - `stand_kd=3.5`
  - `stand_kd_scale_ankle=1.8`
  - `tilt_qref_bias_abs_max=0.03`
  - `tilt_kp_roll/kd_roll=2.0/0.4`
  - `tilt_kp_pitch/kd_pitch=8.0/1.6`
  - `stand_q_ref_trim` 적용
- 즉 현재는 M7 safecheck용 gain과 M8 disturbance용 gain을 따로 들고 가는 구조가 아니라, 공통 standing baseline 하나를 기준으로 이후 `M11`에서 다시 정밀 튜닝하는 흐름으로 정리했다.

### 2026-03-27 추가 정리 — M8/M9 자동 실행 정리
- `ops/run_m8_pair.sh`와 `ops/run_m8_and_m9.sh`도 현재 config/profile 이름 기준으로 정리했다. M8 disturbance 자동 실행은 더 이상 legacy scenario YAML이나 M8 전용 controller 파일에 기대지 않는다.
- M8 세션 종료 흐름도 줄였다. disturbance는 더 빨리 시작하고, `FALL_EVENT`가 찍히면 tmux 쪽에서 짧게 추가 캡처만 하고 세션을 정리하도록 바꿨다.
- 단, 현재 `M8`의 `enable_tilt_feedback` ON/OFF 비교는 아직 완전히 신뢰하지 않는다. 최신 로그를 보면 `balance_off`도 실제 configure 시점에는 `enable_tilt_feedback=true`로 찍히는 run이 있어, launch override 정합성은 `M11`에서 metrics/tuning과 함께 다시 보기로 했다.
- 따라서 현재 판단은 이렇게 둔다.
  - 완료: native M8/M9 실행 파이프라인 정리
  - 완료: M8/M9 산출물 parser/model이 active baseline 기준으로 읽히게 수정
  - 보류: `enable_tilt_feedback` ON/OFF override 정합성 재검토

### 2026-03-27 추가 정리 — 보조 스크립트/문서 active truth 반영
- `scripts/sim2real/kpi/parsers.py`는 이제 `rb_controller` scenario가 아니라 `standing_controller_baseline.yaml`에서 trim hint를 읽는다.
- `scripts/sim2real/kpi/model.py`는 `start.txt`가 없으면 `ros2_control.log`를 source로 써서 active native run의 controller 파라미터를 읽는다.
- legacy safety 설정 파일은 `scripts/sim2real/config/legacy_rb_controller_safety.yaml`로 이름을 바꿨다. active native stack은 `rb_bringup/config/m*_stack*.yaml`을 기준으로 본다.
- `MASTER_PLAN.md`도 현재 구조 기준으로 다시 맞췄다. `M8/M9` 대표 증빙은 그대로 두고, 이번 native 리팩은 `M10`의 "Estimator / Observer 고도화 + Native Control Stack 고정"으로 흡수했다.
