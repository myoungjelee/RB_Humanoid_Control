# Stage 1 Overview — G1 Stand-Only Validation

## 목적
Stand-only 조건(`velocity_commands = 0`)에서 G1의 baseline 안정성을 검증한다.
Stage1에서는 `zero-action`과 `pose-hold`를 비교해, 서기 안정성 검증의 출발점을 만든다.

## 실험 조건
| 항목 | 값 |
|---|---|
| Tasks | `RB-Stage1-G1-Flat-Stand-v0`, `RB-Stage1-G1-Rough-Stand-v0` |
| Modes | `zero`, `pose` |
| Steps | `200` |
| num_envs | `1` |
| device | `cuda:0` |
| fabric | `True` |

## 조건 유효성 확인
- Flat: `velocity_commands[0] = [0.0, 0.0, 0.0]` 확인
- Rough: `velocity_commands[0] = [0.0, 0.0, 0.0]` 확인

## 핵심 결과
### Zero Baseline Result (Stage1)
| Terrain | Mode | First Done Step | Reason | First Done Terms |
|---|---|---:|---|---|
| Flat | zero | 67 | base_contact | base_contact |
| Rough | zero | 67 | base_contact | base_contact |

Sources (2026-02-20):
- `logs/stage1/summary/flat_zero_s200_20260220-095532_summary.md`
- `logs/stage1/summary/rough_zero_s200_20260220-095840_summary.md`

### Pose-hold
| Task | Mode | first_done | reason |
|---|---|---:|---|
| Flat | pose | pending | pending |
| Rough | pose | pending | pending |

## 해석
- Zero-action은 Flat/Rough 모두 67 step에서 첫 종료가 발생했다.
- 종료 원인은 두 지형 모두 `base_contact`로 일치하며, 무제어 상태의 안정성 한계가 동일하게 드러났다.
- 제어 입력이 0이므로 자세 유지 토크가 없어, 중력/접촉 동역학 하에서 빠르게 넘어지는 baseline으로 해석할 수 있다.
- Zero-action baseline failed at step 67 on both flat and rough terrains due to torso ground contact (`base_contact`).
- This result defines the uncontrolled stability limit and serves as the fixed reference for subsequent pose-hold and gain-sweep experiments.
- Pose-hold는 동일 조건에서 zero-action 대비 생존 step이 증가할 것으로 예상한다(명시적 자세 유지 제어가 들어가기 때문).

## 다음 단계
1. Pose-hold(Flat/Rough) 실행 후 결과표 채우기.
2. termination reason(`base_contact`, `time_out` 등) 추적 정확도 보강.
3. Stage1 필수 3개 실험 스크립트(`freq/pd/disturbance`)로 확장.

## 재현
```bash
python scripts/stage1/run_stage1.py --task flat --mode zero --steps 200 --headless
python scripts/stage1/run_stage1.py --task rough --mode zero --steps 200 --headless
python scripts/stage1/run_stage1.py --task flat --mode pose --steps 200 --headless
python scripts/stage1/run_stage1.py --task rough --mode pose --steps 200 --headless
```
