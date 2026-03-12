# MILESTONE REVALIDATION GUIDE

이 문서는 standalone backend 전환 후 `M1 -> M5`를 다시 검증하기 위한 **실행 표준서**다.

목적:
- old evidence와 final evidence를 분리한다
- milestone별 실행/로그/캡처 규칙을 고정한다
- README/overview/one-pager 수정 전에 새 증빙을 먼저 확보한다

## 0. 원칙

- old 캡처/로그는 삭제하지 않는다. 개발 히스토리로 보관한다.
- final evidence는 **standalone backend 기준**으로 다시 딴다.
- standalone backend는 `isaaclab_assets.robots.unitree.G1_CFG` direct spawn 경로를 기준 자산으로 사용한다.
- 재검증 순서는 아래로 고정한다.

```text
M1 -> M2 -> M3 -> M4(clamp/joint_limit/timeout/tilt) -> M5 baseline
```

- milestone 실행은 `tmuxp` YAML을 사용한다.
- `RUN_ID`는 milestone마다 새로 만든다.
- 로그는 `logs/sim2real/$RUN_ID/...`에 저장한다.
- old 캡처는 `reports/sim2real/images/legacy_backend/`에 보관한다.
- final 캡처는 `reports/sim2real/images/standalone_backend/`에 저장한다.
- `tmuxp` pane 중 Isaac 외 pane는 기본적으로 **15초 대기 후**, `/clock` readiness를 확인하고 작업을 시작한다.
- saved USD 기반 standalone 캡처는 개발 히스토리로만 본다. 현재 재검증 기준 source of truth는 direct spawn G1 asset이다.

## 1. 공통 준비

프로젝트 루트에서 아래로 시작한다.

```bash
cd ~/Projects/RB_Humanoid_Control
source "$HOME/miniconda3/etc/profile.d/conda.sh"
conda activate isaac
command -v tmuxp
```

정상 출력 예:

```text
/home/leemou/.local/bin/tmuxp
```

중요:
- `tmuxp`를 띄우는 셸이 `isaac` conda env 위에 있어야 pane 1의 `python main.py ...`가 `isaaclab`를 정상 import 한다.
- ROS2 pane는 각 YAML 안에서 `/opt/ros/humble/setup.bash`와 `ros2_ws/install/setup.bash`를 따로 source 한다.

## 2. 공통 실행 패턴

예:

```bash
RUN_ID=$(date +%Y%m%d-%H%M%S)_m1_standalone tmuxp load -y ops/tmuxp/m1_sensor.yaml
```

의미:
- `RUN_ID`: 이번 검증 세션 식별자
- `tmuxp load`: milestone용 pane 세팅 자동 실행
- 각 pane 로그는 `logs/sim2real/$RUN_ID/...`에 자동 저장

현재 기준:
- `main.py`는 direct spawn backend를 사용하므로, guide의 `tmuxp` 실행 커맨드는 그대로 사용하면 된다.
- 과거 `g1_stage_zero_drive.usd` 설명은 히스토리 문맥이고, 현재 실행 source of truth는 아니다.

세션 종료:
- `Ctrl+C`로 각 pane 명령 종료
- 필요 시 `tmux kill-session -t <session_name>`

## 3. 마일스톤별 실행/캡처/로그

### M1 — Sensor Pipeline

목표:
- direct spawn standalone world에서 `/clock`, `/rb/joint_states`, `/rb/imu`가 정상 publish 되는지 확인

실행:

```bash
cd ~/Projects/RB_Humanoid_Control
source "$HOME/miniconda3/etc/profile.d/conda.sh"
conda activate isaac
RUN_ID=$(date +%Y%m%d-%H%M%S)_m1_standalone tmuxp load -y ops/tmuxp/m1_sensor.yaml
```

자동 저장 로그:

```text
logs/sim2real/$RUN_ID/m1/isaac.log
logs/sim2real/$RUN_ID/m1/topic_list.txt
logs/sim2real/$RUN_ID/m1/joint_states_hz.txt
logs/sim2real/$RUN_ID/m1/imu_once.txt
logs/sim2real/$RUN_ID/m1/clock_once.txt
```

캡처:
- 파일명: `reports/sim2real/images/standalone_backend/m1_standalone.png`
- 화면 기준:
  - topic list
  - `/rb/joint_states` hz
  - `/rb/imu`, `/clock` echo

통과 기준:
- `/clock`, `/rb/joint_states`, `/rb/imu` 존재
- `joint_states hz`가 살아 있음
- `imu_once.txt`, `clock_once.txt` 정상 기록

### M2 — Controller Loop

목표:
- direct spawn sensor backend 위에서 `rb_controller` 200Hz loop/timing 재검증

실행:

```bash
cd ~/Projects/RB_Humanoid_Control
source "$HOME/miniconda3/etc/profile.d/conda.sh"
conda activate isaac
RUN_ID=$(date +%Y%m%d-%H%M%S)_m2_standalone tmuxp load -y ops/tmuxp/m2_controller.yaml
```

자동 저장 로그:

```text
logs/sim2real/$RUN_ID/m2/isaac.log
logs/sim2real/$RUN_ID/m2/controller.log
logs/sim2real/$RUN_ID/m2/command_raw_hz.txt
logs/sim2real/$RUN_ID/m2/loop_tail.txt
logs/sim2real/$RUN_ID/m2/capture_summary.txt
```

캡처:
- 파일명: `reports/sim2real/images/standalone_backend/m2_controller_standalone.png`
- 화면 기준:
  - `capture_summary.txt` 출력 pane 사용
  - `rb_controller started`
  - `loop_stats`
  - `/rb/command_raw` hz

통과 기준:
- `rb_controller started: rate=200.0Hz`
- `loop_stats` 출력 존재
- `/rb/command_raw` 200Hz 근처

### M3 — Command Apply

목표:
- `/rb/command_raw`가 direct spawn standalone world articulation에 적용되는지 확인

실행:

```bash
cd ~/Projects/RB_Humanoid_Control
source "$HOME/miniconda3/etc/profile.d/conda.sh"
conda activate isaac
RUN_ID=$(date +%Y%m%d-%H%M%S)_m3_standalone tmuxp load -y ops/tmuxp/m3_command.yaml
```

자동 저장 로그:

```text
logs/sim2real/$RUN_ID/m3/isaac.log
logs/sim2real/$RUN_ID/m3/controller.log
logs/sim2real/$RUN_ID/m3/command_raw_hz.txt
logs/sim2real/$RUN_ID/m3/js_before.txt
logs/sim2real/$RUN_ID/m3/js_after.txt
logs/sim2real/$RUN_ID/m3/joint_diff.txt
```

캡처:
- 파일명: `reports/sim2real/images/standalone_backend/m3_command_standalone.png`
- 화면 기준:
  - `command_apply=True` 성격의 graph/heartbeat
  - controller 시작
  - `/rb/command_raw` hz
  - `joint_diff.txt` 일부

통과 기준:
- `/rb/command_raw` 200Hz 근처
- `joint_diff.txt`에 관절 변화 존재

### M4 — Safety

목표:
- direct spawn backend에서 `CLAMP`, `JOINT_LIMIT`, `TIMEOUT`, `TILT` reason 재증빙

주의:
- `M4`는 scenario별로 따로 실행한다
- 각 scenario는 독립 `tmuxp` YAML을 사용한다

#### M4-1 Clamp

실행:

```bash
cd ~/Projects/RB_Humanoid_Control
source "$HOME/miniconda3/etc/profile.d/conda.sh"
conda activate isaac
RUN_ID=$(date +%Y%m%d-%H%M%S)_m4_clamp_standalone tmuxp load -y ops/tmuxp/m4_clamp.yaml
```

자동 저장 로그:

```text
logs/sim2real/$RUN_ID/m4/clamp/isaac.log
logs/sim2real/$RUN_ID/m4/clamp/launch.log
logs/sim2real/$RUN_ID/m4/clamp/reasons.txt
logs/sim2real/$RUN_ID/m4/clamp/loop_tail.txt
```

캡처:
- 파일명: `reports/sim2real/images/standalone_backend/m4_clamp_standalone.png`
- 기준: `reason=CLAMP`

#### M4-2 Joint Limit

실행:

```bash
cd ~/Projects/RB_Humanoid_Control
source "$HOME/miniconda3/etc/profile.d/conda.sh"
conda activate isaac
RUN_ID=$(date +%Y%m%d-%H%M%S)_m4_joint_limit_standalone tmuxp load -y ops/tmuxp/m4_joint_limit.yaml
```

자동 저장 로그:

```text
logs/sim2real/$RUN_ID/m4/joint_limit/isaac.log
logs/sim2real/$RUN_ID/m4/joint_limit/launch.log
logs/sim2real/$RUN_ID/m4/joint_limit/reasons.txt
logs/sim2real/$RUN_ID/m4/joint_limit/loop_tail.txt
```

캡처:
- 파일명: `reports/sim2real/images/standalone_backend/m4_joint_limit_standalone.png`
- 기준: `reason=JOINT_LIMIT`

#### M4-3 Timeout

실행:

```bash
cd ~/Projects/RB_Humanoid_Control
source "$HOME/miniconda3/etc/profile.d/conda.sh"
conda activate isaac
RUN_ID=$(date +%Y%m%d-%H%M%S)_m4_timeout_standalone tmuxp load -y ops/tmuxp/m4_timeout.yaml
```

주의:
- 현재 구현에서 `TIMEOUT`은 **`/rb/joint_states` 입력 stale** 기준이다.
- 따라서 재현은 `rb_controller_node` 종료가 아니라 **Isaac/main.py를 `timeout`으로 종료시켜 센서 입력을 끊는 방식**으로 한다.

자동 저장 로그:

```text
logs/sim2real/$RUN_ID/m4/timeout/isaac.log
logs/sim2real/$RUN_ID/m4/timeout/launch.log
logs/sim2real/$RUN_ID/m4/timeout/reasons.txt
logs/sim2real/$RUN_ID/m4/timeout/loop_tail.txt
```

캡처:
- 파일명: `reports/sim2real/images/standalone_backend/m4_timeout_standalone.png`
- 기준: `reason=TIMEOUT`

#### M4-4 Tilt

실행:

```bash
cd ~/Projects/RB_Humanoid_Control
source "$HOME/miniconda3/etc/profile.d/conda.sh"
conda activate isaac
RUN_ID=$(date +%Y%m%d-%H%M%S)_m4_tilt_standalone tmuxp load -y ops/tmuxp/m4_tilt.yaml
```

자동 저장 로그:

```text
logs/sim2real/$RUN_ID/m4/tilt/isaac.log
logs/sim2real/$RUN_ID/m4/tilt/launch.log
logs/sim2real/$RUN_ID/m4/tilt/reasons.txt
logs/sim2real/$RUN_ID/m4/tilt/loop_tail.txt
```

캡처:
- 파일명: `reports/sim2real/images/standalone_backend/m4_tilt_standalone.png`
- 기준: `reason=TILT`

M4 통과 기준:
- 각 scenario에서 해당 reason이 최소 1회 보임

### M5 — Baseline

목표:
- direct spawn standalone backend 기준 `M5 baseline 60초` 재확보
- baseline은 `stand controller off`, 즉 `signal_mode=zero` 상태를 뜻한다
- `stand_pd` 기본 실행은 별도 시나리오(`stand_pd_default.yaml`)로 분리한다

실행:

```bash
cd ~/Projects/RB_Humanoid_Control
source "$HOME/miniconda3/etc/profile.d/conda.sh"
conda activate isaac
RUN_ID=$(date +%Y%m%d-%H%M%S)_m5_baseline_standalone tmuxp load -y ops/tmuxp/m5_baseline.yaml
```

자동 저장 로그:

```text
logs/sim2real/$RUN_ID/m5/isaac.log
logs/sim2real/$RUN_ID/m5/m5_baseline.log
logs/sim2real/$RUN_ID/m5/start.txt
logs/sim2real/$RUN_ID/m5/reason_count.txt
logs/sim2real/$RUN_ID/m5/loop_tail.txt
```

캡처:
- 파일명: `reports/sim2real/images/standalone_backend/m5_baseline_standalone.png`
- 화면 기준:
  - `rb_controller started: ... signal_mode=zero`
  - `reason_count`
  - `loop_tail`

통과 기준:
- `signal_mode=zero` baseline 60초 로그 확보
- `reason_count.txt`, `loop_tail.txt` 생성

## 4. 재검증 후 문서 수정 순서

마일스톤 재검증이 끝난 뒤 아래 순서로 문서를 수정한다.

```text
1. STATUS.md
2. reports/sim2real/overview.md
3. reports/sim2real/ONE_PAGER.md
4. README.md
```

이유:
- `README`는 final evidence를 엮는 문서다
- 먼저 새 증빙이 있어야 한다

## 5. 운영 메모

- `M1/M3/M4/M5` 캡처는 final evidence 기준으로 새로 만든다
- `M2` old 캡처는 재사용 가능하지만, 일관성을 위해 standalone 기준으로 다시 따는 것을 권장한다
- 모든 old evidence는 내부 개발 히스토리로 보관한다
- 실행 중 `isaaclab` import 에러가 나면 `tmuxp`를 띄운 현재 셸에서 `conda activate isaac`가 빠진 경우를 먼저 확인한다
