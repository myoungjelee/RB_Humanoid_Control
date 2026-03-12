# ROADMAP SIMPLE

## 목표
- 로보티즈 휴머노이드 제어 엔지니어 지원용 메인 포트폴리오 완성
- 실기체 없이도 Isaac Sim + ROS2 기반으로 휴머노이드 제어 구조를 직접 구현/검증

## 큰 원칙
- Gym/IsaacLab Stage1은 baseline/archive로 유지
- 메인 트랙은 standalone `World.step()` + ROS2
- 모델은 G1 유지
- `saved USD`는 메인 경로가 아니라 debug/reference artifact로만 사용
- 메인 source of truth는 original G1 direct spawn standalone

## 마일스톤
- M0: 인터페이스/조인트 ordering/frame/command 고정
- M1: 센서 파이프라인 검증
- M2: controller loop / dt-jitter 검증
- M3: command apply 검증
- M4: safety 검증
- M5: stand stabilization
- M6: KPI / summary 자동화
- M7+: 문서/영상/포트폴리오 마무리

## M5 정의
### baseline
- `baseline_zero`
- disturbance를 가했을 때 controller OFF 상태의 반응 확인

### stand
- `stand_pd_default`
- 같은 disturbance에서 controller ON 상태의 반응 확인

### 핵심 메시지
- controller OFF -> 외란에 더 취약
- controller ON  -> 같은 외란에서 더 안정

## 현재 전략
- G1 유지
- Atlas 보류
- disturbance A/B 추가
- KPI summary 추가
- state_estimator는 M5 이후 MVP로만 검토

## 현재 단계
- M1~M4 standalone 재검증 완료
- direct spawn standalone 전환 작업 진행 중
- 이후 M5 disturbance 비교로 넘어갈 예정

## 지원 일정
- 공격적 목표: 3/18
- 현실적 목표: 3/19
- 안전 마감: 3/21
