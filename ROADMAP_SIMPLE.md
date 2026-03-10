# RB_Humanoid_Control -- 단계별 한줄 정리

---

## 일정 스냅샷 (기준일: 2026-03-10)

- 원래 목표: **Stretch 2026-03-19**, 최종 마감: **2026-03-26**
- 현재 완료: **M0, M1, M2, M3, M4**
- 현재 진행: **M5**
- 현재 페이스: **계획 대비 약간 빠름**

### 마일스톤 일정(요약)

- 2026-03-15: M5 완료 목표 (stand 안정화)
- 2026-03-19: M6 완료 목표 (재지원 게이트, Stretch)
- 2026-03-26: 최종 데드라인

---

## M0

### 인터페이스/주기 기준 고정

- command mode(topic/frame/joint order) 고정
- control rate(200Hz) / sim dt(0.005) 고정

---

## M1

### 센서가 잘 들어오는지 확인

- /clock
- /rb/joint_states
- /rb/imu -> 데이터 흐름 검증 단계

---

## M2

### 제어 루프가 잘 도는지 확인

- C++ controller 200Hz timer
- dt_mean / dt_p95 / miss_count -> 타이밍 안정성 증명

---

## M3

### 제어 출력이 모터(시뮬)에 전달되는지 확인

- /rb/command_raw -> Isaac articulation 연결
- joint_states 변화 확인 -> 입력 -> 물리 -> 출력 루프 검증

---

## M4

### 문제 생기면 안전하게 멈추는지 확인

- torque clamp
- timeout watchdog
- joint limit
- tilt limit -> 안전 레이어 검증

---

## M5

### 정책 없이도 서 있는지 확인

- stand pose 유지 제어(PD/impedance)
- 20~30초 안정 유지 + safety 연동
- KPI 수치 확보 -> 제어 기반 완성 단계

---

## M6

### KPI/리포트 자동화

- raw/summary 자동 생성
- overview/one-pager 갱신
- loop dt/jitter 통계 리포트 포함

---

## M7

### 상태추정 v1 적용

- IMU 기반 roll/pitch 안정 추정
- estimator 출력을 controller 입력으로 연결

---

## M8 (선택 강화)

### RT-Ready 패키지

- executor/스레딩 정책 비교
- 우선순위/지연 계측(cyclictest 등)

---

## M9 (선택 강화)

### Real backend 어댑터

- ros2_control/hardware_interface 연결
- Sim/Real 백엔드 교체 가능 구조 검증

---

## 핵심 철학

제어는 정책과 독립적으로 안정적이어야 한다.
정책은 성능을 높이는 계층이고, 안전과 균형은 제어 루프가 책임진다.
