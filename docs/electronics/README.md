# 전장 · 제어 문서

출처: [2학기 통합본 발표](../presentations/2026-2-integrated/) PART 06–08 — 슬라이드의 표 · 수식 · 코드를 글로 옮겨 정리했습니다.

| 문서 | 내용 | 슬라이드 |
|---|---|---|
| [01 CAN 통합 구동 계획](01-can-integration-plan.md) | 현황, 전장 구성, 시스템 구조, 전원 · 배선, CAN ID 지도, DM4340P · ODrive 프로토콜, 펌웨어 구조 · 상태머신, 제어 법칙, ROS 2 패킷, `config.h`, bring-up 8단계 · 일정, 안전 · 추가 구매 | 55–76 |
| [02 코드 해설 · 원리 · 이론](02-control-principles.md) | CAN 물리층 · 중재, 비트 패킹, MIT 모드(스프링 · 댐퍼), 중력보상, FOC, IMU 자세 추정, 역진자 모델, LQR, 3중 워치독, CRC, ROS 2 오도메트리, 단품 시험 | 77–98 |
| [03 운용 방법](03-operation-manual.md) | 1회 설정, 부호 · 균형각 측정, 켜기 → 밸런싱 → 주행 → 끄기, 비상 대처, 고장 코드, 게인 튜닝, 배터리 점검 | 99–109 |
| [04 단품 시험 진행 · 다음 단계 · 원격 조종](04-bringup-progress.md) | 2026-11 시험 결과(DM 단품 · 좌우 ID · 2개 동시 · 다리 구동), 언어 · 역할, 다음 단계, 노트북 vs 게임패드 조종, failsafe | — |
| [05 고관절 구동 시험 절차 (Khadas · Teensy)](05-hip-drive-test-procedure.md) | 노트북 SSH → Khadas 빌드 · 업로드 → `hip_step_test` 시험 → 로그 플롯, 겪은 문제와 해결 | — |
| [06 고관절 토크 실측 · 중력보상](06-hip-torque-gravity-comp.md) | 실측 토크 = Kp × 오차, 펼침이 접힘보다 어려운 이유, 각도별 중력보상 LUT · 부호 처리 · 확인 순서 | — |

설정 파일: [`host/odrive_can_setup.odrivetool.py`](../../host/odrive_can_setup.odrivetool.py) · [`host/dm4340p_setup.md`](../../host/dm4340p_setup.md)

> 상태 (2026-11-16): DM4340P 좌우 ID 설정 · 2개 동시 구동 · 방향 부호 확인, 바퀴 접지 상태 고관절 접힘 · 펼침 성공. 휠 M0 · M1 캘리브 완료, Teensy USB 끊김 해결. 통합 펌웨어 · ROS 2 패키지 소스는 아직 저장소에 포함되지 않았습니다. 진행은 [04](04-bringup-progress.md).
