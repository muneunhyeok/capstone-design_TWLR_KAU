# 2026-2학기 통합본 — 하드웨어 통합 보고 + CAN 구동 · 해설 · 운용

2조 · 2026.09 – 10 · 슬라이드 110장 · 이전 발표: [1주차](../2026-2-week1/) · [2주차](../2026-2-week2/) · [3주차](../2026-2-week3/)

- 전체 슬라이드 PDF: [`2026-2-integrated.pdf`](2026-2-integrated.pdf) (슬라이드 이미지로 재구성, 원본 `.pptx` 34 MB 는 미커밋)
- 1~3주차 발표를 **주제별로 다시 묶고(PART 01–05)**, CAN 통합 구동 계획 · 코드 원리 · 운용 방법(**PART 06–08**)을 새로 추가한 발표입니다.
- 발표에 들어간 영상 22편 중 21편은 1~3주차 폴더 · [`hardware/media/`](../../../hardware/media/)에 있는 것과 같아 링크로 연결했고, 새로 들어간 **IMU Roll · Pitch · Yaw 출력 영상(슬라이드 51)** 만 [`video/`](video/)에 추가했습니다.
- PART 06–08 의 표 · 수식 · 코드는 글로도 정리했습니다 → **[`docs/electronics/`](../../electronics/)**

## 목차

| PART | 섹션 | 슬라이드 | 내용 | 상세 |
|---|---|---|---|---|
| 00 | 진행 과정 | 1–4 | 어셈블리 영상 · 표지 · 목차 · 1~3주차 → CAN 통합 흐름 | [아래](#00--진행-과정) |
| 01 | 시뮬레이션 | 5–11 | MATLAB Simulink 스쿼트 · 주행 피치, 서보 필요 토크, 모션 시뮬레이션 | [1주차](../2026-2-week1/#01--matlab-simulink) · [2주차](../2026-2-week2/) |
| 02 | 링크부 체결 | 12–17 | MISUMI 부시 · 샤프트, 조인트별 최소 길이, 주문 현황 | [1주차](../2026-2-week1/) · [2주차](../2026-2-week2/) |
| 03 | 하드웨어 설계 · 제작 | 18–26 | 서포터 → 어셈블리 → 링크 · 토르소 3D 프린팅 → **토르소 본체 · 아크릴 제작** → 쇼케이스 | [아래](#13주차-내용-재구성-part-01--05) |
| 04 | 구동각 · 역학 해석 | 27–40 | 운용범위 +20.9° / −21.6°, 정역학 · 기구학 · 동역학 | [3주차](../2026-2-week3/) |
| 05 | 전장 구동 테스트 | 41–54 | 서보 · 인휠 단품, LattePanda, ODrive 휠 구동, BNO085 IMU | [2주차](../2026-2-week2/) · [3주차](../2026-2-week3/) |
| **06** | **CAN 통합 구동 계획** 🆕 | 55–76 | DM4340P · ODrive · IMU → Teensy 500 Hz → ROS 2, 배선 · 프로토콜 · 펌웨어 · 일정 | [electronics/01](../../electronics/01-can-integration-plan.md) |
| **07** | **코드 해설 · 원리 · 이론** 🆕 | 77–98 | CAN · MIT 모드 · FOC · 역진자 · LQR · CRC · ROS 2 | [electronics/02](../../electronics/02-control-principles.md) |
| **08** | **운용 방법** 🆕 | 99–109 | 1회 설정 · 켜기 · 밸런싱 · 주행 · 끄기 · 비상 대처 · 튜닝 | [electronics/03](../../electronics/03-operation-manual.md) |

---

## 00 — 진행 과정

<img src="slides/slide-002.jpg" width="720">

<img src="slides/slide-004.jpg" width="720">

| 날짜 | 발표 | 내용 | 통합본 위치 |
|---|---|---|---|
| 09/07 | [1주차](../2026-2-week1/) | Simulink 스쿼트 · 주행, 부시/샤프트, 서보 · 인휠 구동 영상, 제작 계획 | 1 · 2 · 5장 |
| 09/14 | [2주차](../2026-2-week2/) | CoM · 피치, 모션 시뮬레이션, 부시 사양, 링크 3D 프린팅, 모듈 구동 | 1 · 2 · 3장 |
| 09/21 | [3주차](../2026-2-week3/) | 구동각 한계, 정역학 · 동역학, LattePanda, BNO085 IMU | 3 · 4 · 5장 |
| 10/06 | 영상 · 부품 확정 | 쇼케이스 · 어셈블리 v2 · Cura 영상, 고관절 DM4340P 확정, BOM 1,184,795원 | 3장 |
| 다음 | **이 발표** | DM4340P + ODrive + IMU → Teensy 500 Hz → ROS 2, 원리와 운용 방법 | 6 · 7 · 8장 (신규) |

**오프닝 영상** — 어셈블리 v2 ▶ [영상 (36 s)](../../../hardware/media/assembly-animation-v2.mp4)

---

## 1~3주차 내용 재구성 (PART 01 – 05)

<img src="slides/slide-003.jpg" width="720">

슬라이드 5–54 는 1~3주차 발표 내용을 주제별로 다시 배치한 것입니다. 상세 설명과 영상은 각 주차 폴더에 있고,
여기서는 이번에 새로 들어간 슬라이드만 보입니다.

| 통합본 | 원 발표 | 영상 |
|---|---|---|
| 6–9 Simulink · 서보 필요 토크 | 1주차 01 · 2주차 01 | [1주차 video/](../2026-2-week1/video/) · [2주차 video/](../2026-2-week2/video/) |
| 10–11 Motion simulation | 2주차 | [스쿼트](../2026-2-week2/video/motion-sim-squat.mp4) · [주행·밸런싱](../2026-2-week2/video/motion-sim-drive-balance.mp4) |
| 13–17 링크부 체결 | 1주차 02 · 2주차 02 | — |
| 19–22 서포터 · 조립 · 3D 프린팅 | 1주차 03 · 05 · 2주차 03 · 04 | [링크 조립](../2026-2-week2/video/assembly-links.mp4) |
| 23 토르소 출력 시뮬레이션 | 3주차 01 | [Cura (23 h 42 min · 476 g)](../../../hardware/media/torso-print-sim-cura.mp4) |
| 26 모션 쇼케이스 | 3주차 01 | [SQUAT · ROLL · DRIVE · TURN (42 s)](../../../hardware/media/motion-showcase.mp4) |
| 28–40 구동각 · 정역학 · 동역학 | 3주차 02 | [3주차 video/](../2026-2-week3/video/) |
| 42–54 모듈 구동 · LattePanda · IMU | 2주차 05 · 3주차 03 · 04 | [3주차 video/](../2026-2-week3/video/) · 🆕 [Roll · Pitch · Yaw 출력 (슬라이드 51, 15 s)](video/test-imu-rpy.mp4) |

### 새로 추가된 제작 현황 (슬라이드 24–25)

<img src="slides/slide-024.jpg" width="720">

**토르소 본체** — 출력 완료. 내부에 배터리와 전장품이 들어갈 공간 확보.

<img src="slides/slide-025.jpg" width="720">

**토르소 아크릴 판** 제작 현황.

---

## 06 — CAN 통합 구동 계획 🆕

> 글 정리: [`docs/electronics/01-can-integration-plan.md`](../../electronics/01-can-integration-plan.md) · 설정 파일: [`host/`](../../../host/)

<img src="slides/slide-055.jpg" width="720">

<img src="slides/slide-056.jpg" width="720">

**현황** — 휠(ODrive USB 구동)과 IMU(BNO085 I²C 출력)는 확인, 고관절 DM4340P 는 입고 후 미시험, 통합 · ROS 2 는 설계 단계.
남은 핵심은 Teensy 4.1 한 곳에서 CAN 두 줄과 I²C 를 500 Hz 로 함께 돌리는 것.

<img src="slides/slide-057.jpg" width="720">

**전장 구성** — BOM 전장품의 역할 · 연결 · 설정값. 오른쪽 열 숫자는 펌웨어 `config.h` 와 일치.

<img src="slides/slide-058.jpg" width="720">

**시스템 구조** — 상위 LattePanda(ROS 2) · 하위 Teensy(500 Hz 실시간) · 구동부 3계층. USB 로는 50 Hz 명령과 100 Hz 상태만 주고받아 밸런싱 루프가 리눅스 지연에 영향받지 않게 함.

<img src="slides/slide-059.jpg" width="720">

**전원 · 배선** — 6S 한 줄에서 WAGO 분배, CAN 은 양 끝 120 Ω(전원 OFF 에서 60 Ω 확인). **30 A 퓨즈 · 메인/비상 스위치는 BOM 에 없어 추가 필요.**

<img src="slides/slide-060.jpg" width="720">

**CAN 버스 설계** — 한 버스에 두면 DM 명령 ID 가 ODrive E-stop · Heartbeat ID 와 겹쳐 CAN1(고관절) / CAN2(휠)로 분리. 부하 약 25 %.

<img src="slides/slide-061.jpg" width="720">

**DM4340P 프로토콜** — MIT 모드: 위치 16 bit + 속도 · Kp · Kd · 토크 12 bit 를 8 B 한 프레임에. 패킹 결과는 PC 단위테스트로 바이트 검증.

<img src="slides/slide-062.jpg" width="720">

**DM4340P 초기 설정** — 모터마다 ID · 모드 · 영점을 한 번 저장. **한 개씩 연결해서 설정** → [`host/dm4340p_setup.md`](../../../host/dm4340p_setup.md)

<img src="slides/slide-063.jpg" width="720">

**ODrive CAN 설정** — 3주차 axis0 설정을 2축 + CAN 토크 모드로 확장 → [`host/odrive_can_setup.odrivetool.py`](../../../host/odrive_can_setup.odrivetool.py)

<img src="slides/slide-064.jpg" width="720">

**코드 작성 위치** — 어디서 · 무엇으로 · 어떤 코드를 쓰는지와 저장소에 추가할 폴더 구조.

<img src="slides/slide-065.jpg" width="720">

**Teensy 펌웨어** — 2 ms 마다 받기 → 추정 → 제어 → 보내기. 루프 시간 목표 < 2,200 µs.

<img src="slides/slide-066.jpg" width="720">

**안전 상태머신** — BOOT → IDLE → CALIB → READY → BALANCE, 이상 시 FAULT. ROS 명령 끊김은 고장이 아니라 v = 0 으로 균형 유지.

<img src="slides/slide-067.jpg" width="720">

**코드 ① 고관절** — MIT 프레임 만들기 · 피드백 해석 (`dm_motor.cpp`).

<img src="slides/slide-068.jpg" width="720">

**코드 ② 휠** — ODrive CAN Simple 메시지 + 부팅 시퀀스. 3주차에 odrivetool 에서 손으로 치던 명령을 CAN 프레임으로 옮긴 것.

<img src="slides/slide-069.jpg" width="720">

**코드 ③ 밸런싱 · 고관절** — LQR 휠 토크 + 높이별 중력보상. CAD 질량모델(11.24 kg) 기준 초기 게인, 정격 연속 영역을 지키려 기본 높이 455 mm.

<img src="slides/slide-070.jpg" width="720">

**Teensy ⇄ ROS 2** — USB 시리얼 패킷(A5 5A · CRC16). C++ 과 Python 이 같은 바이트를 만드는지 교차 테스트 통과.

<img src="slides/slide-071.jpg" width="720">

**ROS 2 (LattePanda)** — 노드 · 토픽 구성과 실행 순서.

<img src="slides/slide-072.jpg" width="720">

**코드값 정리** — 실물에서 바꿀 값은 `config.h` 한 곳. 확정 · 측정 · 튜닝으로 구분.

<img src="slides/slide-073.jpg" width="720">

**Bring-up 8단계** — 각 단계의 합격 기준을 넘어야 다음 단계로.

<img src="slides/slide-074.jpg" width="720">

**일정 (안)** — 10/12 ~ 12/13. 전장 · 제어 + 기구 미해결 항목(고관절 허브 재설계, 토르소 실측) 병행.

<img src="slides/slide-075.jpg" width="720">

**안전 · 추가 구매** — 미리 막아야 할 6가지 + BOM 에 없는 부품.

<img src="slides/slide-076.jpg" width="720">

**정리** — 목표는 쇼케이스의 SQUAT · ROLL · DRIVE · TURN 을 실물 TWLR 로 재현.

---

## 07 — 코드 해설 · 원리 · 이론 🆕

> 글 정리: [`docs/electronics/02-control-principles.md`](../../electronics/02-control-principles.md)

<img src="slides/slide-077.jpg" width="720">

| 슬라이드 | 주제 | 핵심 |
|---|---|---|
| <img src="slides/slide-078.jpg" width="240"> | 제어 루프 한눈에 | 휠은 Teensy 가 LQR 로 직접, 고관절은 DM4340P 내부 위치 루프 |
| <img src="slides/slide-079.jpg" width="240"> | 코드 파일 지도 | 파일마다 하는 일 하나 — 실행 위치 · 핵심 이름 · 해설 슬라이드 |
| <img src="slides/slide-080.jpg" width="240"> | CAN 물리층 | 두 선의 전압 차로 0/1, 꼬아서 잡음 상쇄, 양 끝 120 Ω |
| <img src="slides/slide-081.jpg" width="240"> | CAN 프레임 · 중재 | ID 가 작을수록 우선, 동시에 보내도 데이터가 깨지지 않음 |
| <img src="slides/slide-082.jpg" width="240"> | FlexCAN_T4 코드 | 인터럽트는 큐에 복사만, 해석은 loop 에서 |
| <img src="slides/slide-083.jpg" width="240"> | 고정소수점 · 비트 패킹 | 해상도 = 범위 ÷ (2ⁿ − 1), 위치 0.022° · 토크 0.014 N·m |
| <img src="slides/slide-084.jpg" width="240"> | MIT 모드 = 스프링 · 댐퍼 | 서 있으면 관성 10 배 → Kd 1.5 → 3.0 |
| <img src="slides/slide-085.jpg" width="240"> | 중력보상 · LUT · 변화율 제한 | 455 mm 에서 τff 7.18 N·m, 0.35 rad/s 로 천천히 |
| <img src="slides/slide-086.jpg" width="240"> | 휠 토크 제어 (FOC) | τ = Kt · Iq, 오프셋 6° 틀리면 토크 0 → 부팅마다 캘리브 |
| <img src="slides/slide-087.jpg" width="240"> | ODrive CAN 메시지 | ID = node << 5 \| cmd, 값 = IEEE 754 리틀엔디안 |
| <img src="slides/slide-088.jpg" width="240"> | ODrive 설정 스크립트 | 줄마다 무엇을 정하고 왜 그 값인가 |
| <img src="slides/slide-089.jpg" width="240"> | IMU 자세 추정 | Game Rotation Vector(지자기 제외), ω 는 자이로 직접 |
| <img src="slides/slide-090.jpg" width="240"> | 바퀴 달린 역진자 | 열린 루프 극점 ±7.59 → 0.09 s 마다 기울기 2 배 |
| <img src="slides/slide-091.jpg" width="240"> | LQR | Q = diag(5, 2, 200, 5), R = 1 → K = −2.24 −4.10 −28.4 −4.59 |
| <img src="slides/slide-092.jpg" width="240"> | 밸런싱 코드 한 줄씩 | 따라오기가 곧 주행, 와인드업 0.3 m 제한 |
| <img src="slides/slide-093.jpg" width="240"> | 메인 루프 · 상태머신 코드 | 정확히 500 Hz, enable 마지막 값 유지 |
| <img src="slides/slide-094.jpg" width="240"> | 3중 워치독 | ROS 250 ms · DM 100 ms · ODrive 0.2 s |
| <img src="slides/slide-095.jpg" width="240"> | 시리얼 패킷 · CRC | 실제 22 B 패킷, CRC 불일치면 2 B 버리고 재동기 |
| <img src="slides/slide-096.jpg" width="240"> | ROS 2 브리지 코드 | 50 Hz 송신 타이머 + 별도 수신 스레드 |
| <img src="slides/slide-097.jpg" width="240"> | ROS 2 원리 · 오도메트리 | 차동구동 적분 → /odom, 나중에 SLAM 이 보정 |
| <img src="slides/slide-098.jpg" width="240"> | 단품 시험 스케치 | 키 하나 = CAN 프레임 하나 |

---

## 08 — 운용 방법 🆕

> 글 정리: [`docs/electronics/03-operation-manual.md`](../../electronics/03-operation-manual.md) — BOM 추가 부품을 모두 단 상태를 가정

<img src="slides/slide-099.jpg" width="720">

| 슬라이드 | 주제 | 핵심 |
|---|---|---|
| <img src="slides/slide-100.jpg" width="240"> | 최종 구성 | 퓨즈 · 비상 스위치 · 종단저항 반영, 2인 1조 |
| <img src="slides/slide-101.jpg" width="240"> | 처음 한 번만 하는 설정 | 모터 ID → ODrive → 펌웨어 → ROS 2 |
| <img src="slides/slide-102.jpg" width="240"> | 방향 · 부호 · 균형각 측정 | 부호가 틀리면 즉시 튀어 나감 → 서포터 위에서 |
| <img src="slides/slide-103.jpg" width="240"> | 전원 켜기 → READY | 서포터 위, 바퀴가 공중에 뜬 상태에서 시작 |
| <img src="slides/slide-104.jpg" width="240"> | 밸런싱 시작 · 주행 | 균형점 근처에서 balance → 천천히 손을 놓음 |
| <img src="slides/slide-105.jpg" width="240"> | 정지 · 종료 순서 | 반드시 서포터 위에서 끔 |
| <img src="slides/slide-106.jpg" width="240"> | 비상 상황 대처 | 확신이 없으면 비상 스위치 |
| <img src="slides/slide-107.jpg" width="240"> | 고장 코드 읽는 법 | /twlr/faults 이름 → 원인 → 조치 |
| <img src="slides/slide-108.jpg" width="240"> | 게인 튜닝 순서 | θ_eq → 자세 → 위치 · 속도 → 선회 → 고관절 |
| <img src="slides/slide-109.jpg" width="240"> | 배터리 · 점검 | 21.0 V 아래 운용 중단, 보관 22.8 V |
