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
### Zero-action
| Task | Mode | first_done | reason | episode_end_count | avg_reward_per_step | elapsed_s |
|---|---|---:|---|---:|---:|---:|
| Flat | zero | 67 | unknown | 1 | -0.040772 | 2.977 |
| Rough | zero | 67 | unknown | 1 | -0.020352 | 2.985 |

Sources:
- `logs/stage1/summary/flat_zero_s200_20260219-214152_summary.md`
- `logs/stage1/summary/rough_zero_s200_20260219-214057_summary.md`

### Pose-hold
| Task | Mode | first_done | reason |
|---|---|---:|---|
| Flat | pose | pending | pending |
| Rough | pose | pending | pending |

## 해석
- Zero-action은 Flat/Rough 모두 67 step에서 첫 종료가 발생했다.
- 제어 입력이 0이므로 자세 유지 토크가 없어, 중력/접촉 동역학 하에서 빠르게 넘어지는 baseline으로 해석할 수 있다.
- 현재 `first_done_reason`은 `unknown`으로 기록되므로, Stage1 후속에서 termination reason 매핑을 보강해야 한다.
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
