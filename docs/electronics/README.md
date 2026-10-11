# 전장 · 제어 문서

출처: [2학기 통합본 발표](../presentations/2026-2-integrated/) PART 06–08 — 슬라이드의 표 · 수식 · 코드를 글로 옮겨 정리했습니다.

| 문서 | 내용 | 슬라이드 |
|---|---|---|
| [01 CAN 통합 구동 계획](01-can-integration-plan.md) | 현황, 전장 구성, 시스템 구조, 전원 · 배선, CAN ID 지도, DM4340P · ODrive 프로토콜, 펌웨어 구조 · 상태머신, 제어 법칙, ROS 2 패킷, `config.h`, bring-up 8단계 · 일정, 안전 · 추가 구매 | 55–76 |
| [02 코드 해설 · 원리 · 이론](02-control-principles.md) | CAN 물리층 · 중재, 비트 패킹, MIT 모드(스프링 · 댐퍼), 중력보상, FOC, IMU 자세 추정, 역진자 모델, LQR, 3중 워치독, CRC, ROS 2 오도메트리, 단품 시험 | 77–98 |
| [03 운용 방법](03-operation-manual.md) | 1회 설정, 부호 · 균형각 측정, 켜기 → 밸런싱 → 주행 → 끄기, 비상 대처, 고장 코드, 게인 튜닝, 배터리 점검 | 99–109 |

설정 파일: [`host/odrive_can_setup.odrivetool.py`](../../host/odrive_can_setup.odrivetool.py) · [`host/dm4340p_setup.md`](../../host/dm4340p_setup.md)

> 상태 (2026-10): 휠 · IMU 단품 구동 확인, 고관절 DM4340P 입고 후 미시험. 통합 펌웨어 · ROS 2 패키지 소스는 아직 저장소에 포함되지 않았습니다.
