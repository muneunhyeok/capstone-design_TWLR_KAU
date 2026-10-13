# 시뮬레이션 문서

Gazebo Harmonic 균형 시뮬레이션(`sim/`)의 구성 · 값 · 검증 결과입니다. 출처: 2026-10-12 공유된 `twlr_sim_v1`(패키지 2개 + 설치·실행 메모 + 「TWLR 질량·무게중심 정리 (URDF 기준)」) 와 2026-10-13 검증.

| 문서 | 내용 |
|---|---|
| [01 Gazebo 시뮬레이션 구성 · 실행 · 값](01-gazebo-simulation.md) | 설치 · 실행, 런치 인자, ROS 토픽, 좌표계, URDF 치수 · 질량 · 무게중심, 임시값, 제어기 구조 · 게인 |
| [02 검증 결과 · CAD 와의 차이 · 열린 항목](02-verification-and-open-items.md) | Gazebo 없이 확인한 것, 제어기 여유, URDF(9.62 kg) 와 CAD(11.24 kg) 불일치, LQR 모델과의 차이, 첫 실행 체크 |

코드: [`sim/`](../../sim/) · 재현 스크립트와 결과: [`sim/verify/`](../../sim/verify/)

> **상태 (2026-10-13)** — 패키지 코드와 URDF 는 읽고 계산으로 확인했지만, 이 저장소의 검증은 **실제 Gazebo 구동 결과가 아닙니다.** 처음 띄울 때 확인할 항목은 [02 §6](02-verification-and-open-items.md#6-gazebo-첫-실행-체크).
