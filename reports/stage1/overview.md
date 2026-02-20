# Stage1 Overview — Stand-Only Validation (G1 / IsaacLab)

## 1) 목적
Stage1은 stand-only(velocity command=0) 조건에서 `zero` 기준선을 확정하고,
termination/failure mode를 재현 가능하게 기록하는 검증 단계다.

## 2) 실행 환경(재현 정보)
- Repo: `RB_Humanoid_Control`
- IsaacLab: `0.54.3`
- Device: `cuda:0`
- Task IDs:
  - Flat: `RB-Stage1-G1-Flat-Stand-v0`
  - Rough: `RB-Stage1-G1-Rough-Stand-v0`
- Steps: `200`
- num_envs: `1`
- Runner: `scripts/stage1/run_stage1.py`
- Summary output: `logs/stage1/summary/*_summary.md`
- Raw kit log: `logs/stage1/raw/*.log`

## 3) 결과 요약(표)
| Terrain | Mode | executed_steps | first_done_step | first_done_reason | first_done_terms |
|---|---|---:|---:|---|---|
| Flat | zero | 67 | 67 | base_contact | base_contact |
| Rough | zero | 67 | 67 | base_contact | base_contact |
| Flat | pose | pending | pending | pending | pending |
| Rough | pose | pending | pending | pending | pending |

### 3.1 한 줄 해석
- `zero`는 Flat/Rough 모두 67 step에서 `base_contact`로 종료되어 uncontrolled stability baseline이 고정됐다.
- 이 baseline은 이후 `pose-hold`, gain sweep, frequency sweep 비교의 기준점으로 사용한다.

## 4) 아티팩트(링크 허브)
### 4.1 Summary
- Flat zero: `logs/stage1/summary/flat_zero_s200_20260220-121311_summary.md`
- Rough zero: `logs/stage1/summary/rough_zero_s200_20260220-121937_summary.md`

### 4.2 Videos
- `logs/stage1/videos/flat_zero_trim.mp4`
- `logs/stage1/videos/rough_zero_trim.mp4`

### 4.3 Images
- `reports/stage1/images/flat_zero.png`
- `reports/stage1/images/rough_zero.png`

### 4.4 Figures (pending)
- `reports/stage1/figures/pd_sweep.png`
- `reports/stage1/figures/freq_sweep.png`

## 5) 실험 설계 노트
- Mode 정의:
  - `zero`: action=0
  - `pose`: pose-hold action (구현 완료, 결과 취합 전)
- Termination:
  - `base_contact` (torso_link contact)
- 측정 지표:
  - `executed_steps`
  - `first_done_step`
  - `first_done_reason`
  - `first_done_terms`
  - `avg_reward_per_step`

## 6) 재현 커맨드
```bash
python scripts/stage1/run_stage1.py --task flat --mode zero --steps 200
python scripts/stage1/run_stage1.py --task rough --mode zero --steps 200
python scripts/stage1/run_stage1.py --task flat --mode pose --steps 200
python scripts/stage1/run_stage1.py --task rough --mode pose --steps 200
```

## 7) 다음 단계
1. pose-hold(Flat/Rough) 결과를 동일 표에 채워 zero 대비 개선폭 확정.
2. PD/frequency sweep 스크립트 결과를 `reports/stage1/figures/`에 연결.
3. disturbance recovery 결과까지 추가해 Stage1 패키지 완성.
