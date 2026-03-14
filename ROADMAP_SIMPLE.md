# ROADMAP SIMPLE

## 프로젝트 한 줄

실기체 없이도 `Isaac Sim + ROS2` 기반 휴머노이드 제어 시스템을 직접 만들고,
재현 가능한 증빙으로 포트폴리오를 완성한다.

## 큰 원칙

- Gym/IsaacLab Stage1은 baseline/archive로 유지한다.
- 메인 트랙은 `standalone World.step() + ROS2`다.
- 로봇 모델은 `G1`으로 고정한다.
- `saved USD`는 debug/reference artifact로만 둔다.
- 메인 source of truth는 `original G1 direct spawn standalone`이다.

## 지금 어디까지 왔는가

- `M1~M4` standalone 재검증 완료
- `M5` controller-only no-disturbance standing 확보
- 핵심 원인은 `imu_link -> control frame` 해석 불일치였고, observer 보정으로 해결
- 다음 우선순위는 `M7 세이프티 적용 스탠딩 -> M8 외란 A/B 비교 -> M9 KPI 자동화`

---

## M0 ~ M6: 시스템 구축 단계

| 마일스톤 | 무엇을 하는가 | 왜 필요한가 | 목표 |
| --- | --- | --- | --- |
| `M0 인터페이스 고정` | 토픽, 프레임, 조인트 ordering, command mode를 고정한다. | 입출력 계약이 흔들리면 이후 실험 결과가 비교 불가능해진다. | 제어 경로의 기준 인터페이스를 잠근다. |
| `M1 센서 파이프라인` | `/clock`, `/rb/joint_states`, `/rb/imu`가 정상 publish 되는지 확인한다. | controller와 safety는 센서 입력이 살아 있어야 의미가 있다. | ROS2 입력 파이프라인이 정상 동작함을 증명한다. |
| `M2 제어 루프` | `rb_controller`가 고정 주기로 돌고 `dt` 통계를 남기는지 확인한다. | 제어기는 일정한 주기로 안정적으로 돌아야 한다. | `200Hz` controller loop와 dt/jitter 관측을 증명한다. |
| `M3 명령 적용` | `/rb/command_raw`가 실제 articulation motion으로 이어지는지 확인한다. | 토픽이 나온다는 것만으로는 부족하고, 명령이 실제 state를 바꿔야 한다. | `ROS2 command -> sim motion` 경로를 증명한다. |
| `M4 세이프티 계층` | `CLAMP`, `JOINT_LIMIT`, `TIMEOUT`, `TILT`, `VELOCITY_LIMIT`를 구축하고 확인한다. | 실무형 제어 시스템은 성능뿐 아니라 실패 시 안전 동작이 중요하다. | safety가 실제 런타임 계층임을 증명한다. |
| `M5 스탠딩 안정화` | no-disturbance standing을 확보하고 pose/gain/actuator/IMU frame을 분리 진단한다. | disturbance 전에 기본 standing이 살아 있어야 이후 비교가 의미 있다. | `controller-only standing` 확보, `imu_link -> control frame` 문제를 observer 보정(`imu_frame_mode=g1_imu_link`)으로 해결한다. |
| `M6 증빙 / 아티팩트 인프라` | run log, params, sync marker, fall event, loop window를 자동으로 남긴다. | 실험은 “돌렸다”가 아니라 나중에 다시 설명 가능해야 한다. | `logs/`, `summary/`, `artifacts/` 구조를 갖춘다. |

---

## M7 ~ M10: 현재 마감선 단계

| 마일스톤 | 무엇을 하는가 | 왜 필요한가 | 목표 |
| --- | --- | --- | --- |
| `M7 세이프티 적용 스탠딩` | `safety_enabled=true` 상태에서도 standing이 유지되는지 본다. | controller-only로 서는 것과 safety까지 포함해 실무형 구조로 서는 것은 다르다. | safety가 정상 동작하면서 standing을 불필요하게 끊지 않는다. |
| `M8 외란 A/B 비교` | 같은 외란에서 controller OFF / ON을 비교한다. | 제어기가 실제 안정성에 기여하는지 보여주는 핵심 증빙이기 때문이다. | OFF보다 ON이 더 오래 버티거나 더 잘 회복함을 증명한다. |
| `M9 KPI 자동화` | standing time, fall reason, disturbance 조건, controller parameter, dt jitter를 자동 요약한다. | 실험 결과를 숫자로 비교하고 run 간 차이를 자동으로 설명할 수 있어야 한다. | run summary와 KPI 비교 자동화를 만든다. |
| `M10 포트폴리오 패키징` | README, overview, one-pager, figures, video를 하나의 스토리로 묶는다. | 기술 작업이 끝나도 전달이 약하면 포트폴리오 가치가 떨어진다. | 지원서에 바로 넣을 수 있는 패키지 형태로 정리한다. |

---

## M11 ~ M16: 이후 확장 단계

| 마일스톤 | 무엇을 하는가 | 왜 필요한가 | 목표 |
| --- | --- | --- | --- |
| `M11 추정기 / 옵저버 고도화` | IMU extrinsic, state estimation, contact-aware estimation을 고도화한다. | 센서 해석을 controller와 더 명확히 분리해야 확장성이 생긴다. | observer를 독립 책임으로 강화한다. |
| `M12 밸런스 지표` | COM, ZMP, capture point, support polygon margin을 계산한다. | 넘어짐을 더 물리적으로 설명할 수 있어야 한다. | balance failure를 정량 지표로 해석한다. |
| `M13 전신 제어` | joint-space PD에서 task-space control로 확장한다. | torso / CoM / foot task를 동시에 다루는 구조로 가기 위해서다. | WBC 실험 기반을 만든다. |
| `M14 RL 정책 계층` | stabilizing controller 위에 policy / residual / reference layer를 올린다. | 저수준 안정화와 고수준 정책을 분리해야 실험 구조가 깔끔해진다. | low-level + high-level 2층 구조를 만든다. |
| `M15 Sim-to-Real 강건화` | delay, noise, latency, domain randomization을 추가한다. | 실기체 대응 robustness는 깨끗한 sim만으로는 증명되지 않는다. | sim-to-real 간극을 줄이는 강건성 실험을 추가한다. |
| `M16 실험 인프라 고도화` | dashboard, KPI trend, failure taxonomy, auto markdown summary를 만든다. | 실험 수가 많아질수록 결과 관리 자동화가 필요해진다. | 반복 실험을 체계적으로 관리하는 인프라를 완성한다. |

---

## 현재 우선순위

1. M7 - 세이프티 적용 스탠딩
2. M8 - 외란 A/B 비교
3. M9 - KPI 자동화
4. M10 - 포트폴리오 패키징
5. 이후 M11+ 확장
