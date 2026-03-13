# ROADMAP SIMPLE

## 프로젝트 한 줄
실기체 없이도 `Isaac Sim + ROS2` 기반 휴머노이드 제어 시스템을 직접 만들고, 재현 가능한 증빙으로 포트폴리오를 완성한다.

## 큰 원칙
- Gym/IsaacLab Stage1은 baseline/archive로 유지한다.
- 메인 트랙은 `standalone World.step() + ROS2`다.
- 로봇 모델은 `G1`으로 고정한다.
- `saved USD`는 debug/reference artifact로만 둔다.
- 메인 source of truth는 `original G1 direct spawn standalone`이다.

## 지금 어디까지 왔는가
- `M1~M4` standalone 재검증 완료
- `M5` 진입 전제인 no-disturbance `stand`는 아직 미완료
- 다음 핵심 단계는 `M5 stand sanity -> baseline_zero vs stand_pd_default 비교`

## 마일스톤 정리

### M0 - Interface Freeze

무엇을 하는가:
- 토픽, 조인트 ordering, frame 이름, command 인터페이스, dt를 고정한다.

왜 하는가:
- 이게 흔들리면 이후 controller, safety, evidence가 전부 흔들린다.

목표:
- 제어 경로의 입출력 계약을 고정한다.

### M1 - Sensor Pipeline

무엇을 하는가:
- `/clock`, `/rb/joint_states`, `/rb/imu`가 정상 publish 되는지 확인한다.

왜 하는가:
- controller와 safety는 결국 센서 입력이 살아 있어야 의미가 있다.

목표:
- ROS2 입력 파이프라인이 살아 있음을 증명한다.

### M2 - Controller Loop

무엇을 하는가:
- `rb_controller`가 고정 주기로 돌고 `/rb/command_raw`를 publish 하는지 확인한다.

왜 하는가:
- 제어기는 단순히 존재하는 것보다 일정한 주기로 안정적으로 돌아야 한다.

목표:
- `200Hz` controller loop와 command publish를 증명한다.

### M3 - Command Apply

무엇을 하는가:
- `/rb/command_raw`가 실제 articulation motion으로 이어지는지 확인한다.

왜 하는가:
- 토픽이 나온다는 것만으로는 부족하고, 그 명령이 실제 robot state를 바꿔야 한다.

목표:
- `ROS2 command -> sim motion` 경로를 증명한다.

### M4 - Safety

무엇을 하는가:
- `CLAMP`, `JOINT_LIMIT`, `TIMEOUT`, `TILT`가 실제로 발동하는지 확인한다.

왜 하는가:
- 실무형 제어 시스템은 제어 성능만이 아니라 실패 시 안전 동작이 중요하다.

목표:
- safety가 문서가 아니라 실제 런타임 동작임을 증명한다.

### M5 - Disturbance Comparison

무엇을 하는가:
- 먼저 no-disturbance `stand`를 확보하고, 그 다음 `baseline_zero`와 `stand_pd_default`를 같은 disturbance에서 비교한다.

왜 하는가:
- `M1~M4`는 제어 경로가 존재함을 보여준다.
- `M5`는 그 제어 경로가 실제 안정성에 기여하는지 보여준다.

목표:
- `controller OFF보다 controller ON이 같은 외란에서 더 안정적이다`를 증명한다.

비교 기준:
- 같은 robot
- 같은 시작 자세
- 같은 disturbance 시점
- 같은 disturbance 방향
- 같은 disturbance 크기 또는 방식

### M6 - KPI / Summary

무엇을 하는가:
- 로그, 캡처, 비교 결과를 정리해 summary와 KPI로 묶는다.

왜 하는가:
- 포트폴리오는 실행만이 아니라 결과를 해석 가능하게 정리해야 한다.

목표:
- README, one-pager, overview에 넣을 수 있는 정리본을 만든다.

### M7 - Portfolio Finish

무엇을 하는가:
- README, one-pager, overview, 대표 이미지, 최종 영상을 마무리한다.

왜 하는가:
- 기술 작업이 끝나도 전달이 약하면 포트폴리오 가치가 떨어진다.

목표:
- 지원서에 바로 넣을 수 있는 형태로 패키징한다.

## M5 지금 해야 할 일

### 1. stand sanity

무엇을 확인하는가:
- `stand_pd_sanity`가 외란 없이 기본 stand를 20~30초 정도 유지하는지 본다.

왜 하는가:
- controller ON 상태가 최소한 기본 stand는 가능해야 다음 비교가 의미 있다.

운영 기준:
- `stand_pd_default`가 baseline과 같은 시점에 무너지면 disturbance 실험을 중단하고 `stand_pd_sanity`로 먼저 튜닝한다.

### 2. baseline sanity

무엇을 확인하는가:
- `baseline_zero` 상태를 한 번 본다.

왜 하는가:
- baseline의 기본 반응을 먼저 알아야 disturbance 비교 해석이 쉬워진다.

주의:
- baseline은 보통 `ROS2 stack은 유지하되 signal_mode=zero`로 두는 것이 맞다.
- 프로세스를 아예 끄는 것보다 비교 조건이 더 공정하다.

### 3. disturbance spec 고정

무엇을 고정하는가:
- target body
- direction
- trigger timing
- magnitude or method

왜 하는가:
- `baseline_zero`와 `stand_pd_default`를 같은 조건에서 비교해야 하기 때문이다.

### 4. 본실험

무엇을 하는가:
- 같은 disturbance를 `baseline_zero`와 `stand_pd_default`에 각각 적용한다.

무엇을 비교하는가:
- 누가 먼저 무너지는지
- 누가 더 오래 버티는지
- 누가 회복하는지
- safety가 언제 개입하는지

### 5. M5 완료 기준

아래 중 하나가 보이면 된다.
- `baseline_zero`는 무너지는데 `stand_pd_default`는 버틴다
- 둘 다 무너지지만 `stand_pd_default`가 더 오래 버틴다
- `stand_pd_default`가 disturbance 후 더 잘 회복한다

## 현재 전략
- G1 유지
- Atlas 보류
- M5 disturbance 비교 완료 후 KPI/summary로 이동
- state_estimator는 M5 이후 MVP로만 검토

## 지원 일정
- 공격적 목표: `3/18`
- 현실적 목표: `3/19`
- 안전 마감: `3/21`
