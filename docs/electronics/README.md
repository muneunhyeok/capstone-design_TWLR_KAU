# 전장 · 제어 문서

출처: [2학기 통합본 발표](../presentations/2026-2-integrated/) PART 06–08 — 슬라이드의 표 · 수식 · 코드를 글로 옮겨 정리했습니다.

| 문서 | 내용 | 슬라이드 |
|---|---|---|
| [01 CAN 통합 구동 계획](01-can-integration-plan.md) | 현황, 전장 구성, 시스템 구조, 전원 · 배선, CAN ID 지도, DM4340P · ODrive 프로토콜, 펌웨어 구조 · 상태머신, 제어 법칙, ROS 2 패킷, `config.h`, bring-up 8단계 · 일정, 안전 · 추가 구매 | 55–76 |
| [02 코드 해설 · 원리 · 이론](02-control-principles.md) | CAN 물리층 · 중재, 비트 패킹, MIT 모드(스프링 · 댐퍼), 중력보상, FOC, IMU 자세 추정, 역진자 모델, LQR, 3중 워치독, CRC, ROS 2 오도메트리, 단품 시험 | 77–98 |
| [03 운용 방법](03-operation-manual.md) | 1회 설정, 부호 · 균형각 측정, 켜기 → 밸런싱 → 주행 → 끄기, 비상 대처, 고장 코드, 게인 튜닝, 배터리 점검 | 99–109 |
| [**04 실물 밸런싱 v3 (11-22)**](04-balancing-v3-teensy.md) | 실제 구현 구조(ROS 2 미사용), 10월 계획과의 차이표, 제어식, 확정 설정, 실행 순서, 알려진 문제 | — |
| [**05 다음 단계 계획**](05-next-motions-roadmap.md) | 안전 선행 작업, 선회 요레이트 피드백, 스쿼트(펌웨어 수정 지점 · 높이별 토크 표), 롤, ROS 2 브리지 노드 | — |

설정 파일: [`host/odrive_can_setup.odrivetool.py`](../../host/odrive_can_setup.odrivetool.py) · [`host/dm4340p_setup.md`](../../host/dm4340p_setup.md)

> 상태 (2026-11-22): **실물 밸런싱 성공.** 통합 펌웨어와 상위 코드는 [`twlr_balance/`](../../twlr_balance/)에 있습니다. ROS 2 패키지는 아직 없습니다 (계획: [05 §4](05-next-motions-roadmap.md#4-ros-2-로-올리기)).
> 01–03 은 10월 계획 기준이라 CAN 버스 구성 · ODrive 모드 · 노드 번호가 실제와 다릅니다 → [04 §1 차이표](04-balancing-v3-teensy.md#10월-계획-문서01와-달라진-점).
