# tmuxp Revalidation Sessions

standalone backend 전환 후 milestone 재검증용 `tmuxp` 세션 모음이다.

- `ops`는 `operations` 약자다.
- 실행 표준, tmuxp 세션, 재검증 절차 같은 운영 자산을 여기에 둔다.

## 공통 실행 규칙

각 세션은 `RUN_ID`를 외부에서 주입받는다.

예:

```bash
RUN_ID=$(date +%Y%m%d-%H%M%S)_m1_standalone tmuxp load -y ops/tmuxp/m1_sensor.yaml
```

로그는 자동으로 아래 경로에 저장된다.

```text
logs/sim2real/$RUN_ID/
```

## 세션 목록

- `m1_sensor.yaml`
- `m2_controller.yaml`
- `m3_command.yaml`
- `m4_clamp.yaml`
- `m4_joint_limit.yaml`
- `m4_timeout.yaml`
- `m4_tilt.yaml`
- `m5_baseline.yaml`

## 캡처 파일 이름(권장)

- `reports/sim2real/images/standalone_backend/m1_standalone.png`
- `reports/sim2real/images/standalone_backend/m2_controller_standalone.png`
- `reports/sim2real/images/standalone_backend/m3_command_standalone.png`
- `reports/sim2real/images/standalone_backend/m4_clamp_standalone.png`
- `reports/sim2real/images/standalone_backend/m4_joint_limit_standalone.png`
- `reports/sim2real/images/standalone_backend/m4_timeout_standalone.png`
- `reports/sim2real/images/standalone_backend/m4_tilt_standalone.png`
- `reports/sim2real/images/standalone_backend/m5_baseline_standalone.png`

## 순서

1. `m1_sensor.yaml`
2. `m2_controller.yaml`
3. `m3_command.yaml`
4. `m4_clamp.yaml`
5. `m4_joint_limit.yaml`
6. `m4_timeout.yaml`
7. `m4_tilt.yaml`
8. `m5_baseline.yaml`
