# 04 단품 시험 진행 · 다음 단계 · 원격 조종

2026-11-09 작성, **2026-11-16 갱신**. 계획은 [01 CAN 통합 구동 계획](01-can-integration-plan.md), 시험 스케치는 [`firmware/bringup/`](../../firmware/bringup/).

## 1. 현황

| 대상 | 상태 | 비고 |
|---|---|---|
| DM4340P 1개 (CAN1) | **완료** — 피드백 수신, 정 · 역 회전, 회전 후 영점(0xFE) | 초기 no feedback 은 CAN 배선 문제, 수정 후 해결 |
| DM4340P 설정값 | **확인** — CAN ID 1, PMAX 12.5 · VMAX 10 · TMAX 28 | Teensy 로 레지스터(`0x7FF`) 직접 읽음. 계획 문서의 VMAX 8 → 10 으로 수정 |
| DM4340P 좌 · 우 ID | **완료** — L 1 / 0x11, R 2 / 0x12, 저장 후 전원 재투입에도 유지 | `dm_config_tool` `j` 조그로 모터 확인 → `L` / `R`. 레지스터 쓰기 · 저장 실물 검증됨 |
| DM4340P 2개 동시 | **완료** — 한 버스에서 2개 피드백 · 동시 구동 | `dm_dual_test` (429줄 판). 다리 장착 후 `BENCH_SPIN_ALLOWED = false` |
| 방향 부호 | **확인** — 다리 + = 접힘, L `dir` +1 · R `dir` −1 | 오른쪽은 거울 장착. 명령 · 피드백 모두에 곱함 |
| 고관절 다리 구동 | **바퀴 접지 상태 접힘 · 펼침 성공** (2026-11-16) | `hip_step_test` v2 (들기 모드). 전원 24 V 3 A 파워서플라이 |
| 고관절 토크 실측 | 처짐 6° → 2.8 ~ 3.4 N·m, 막힘 9° → 4.5 ~ 5.1 N·m (= Kp 30 × 오차) | [06](06-hip-torque-gravity-comp.md) |
| ODrive 휠 M0 · M1 | 캘리브레이션 완료 | M0 는 `ENCODER_FAILED / NO_RESPONSE` → 엔코더 배선 점검 후 통과. odrivetool(Python), USB |
| Teensy USB | 모터 전원 투입 시 끊김 → 해결 | |
| 시험용 상위 보드 | Khadas VIM4 (Ubuntu 24.04, aarch64) | 계획 문서의 LattePanda 자리. 노트북에서 SSH · scp |

## 2. 언어 · 역할 나누기

| 층 | 어디서 | 언어 | 하는 일 |
|---|---|---|---|
| 하위 제어 | Teensy 4.1 | C++ (Arduino, arduino-cli 로 빌드) | 500 Hz 루프, CAN1 · CAN2 송수신, IMU, 상태추정 · LQR · 고관절, 안전 감시 |
| 상위 | Khadas (ROS 2 Jazzy) | Python (rclpy) | `/cmd_vel` · `/twlr/enable` 받아 USB 패킷 송신, 상태 패킷 → 토픽 |
| 설정 도구 | Khadas / 노트북 | Python (odrivetool), C++ (`dm_config_tool`) | 1회 설정 · 캘리브레이션만. 실행 중에는 안 씀 |

언어보다 **둘 사이 약속**을 고정합니다.

- USB 패킷 형식 — 01 §9: `A5 5A | type | len | payload | CRC16-CCITT`, Command 16 B @ 50 Hz (`seq u16 · enable u8 · balance u8 · v_cmd f32 · yaw_rate f32 · height_mm f32`), Telemetry 92 B @ 100 Hz
- 단위 · 부호 — rad · rad/s · N·m · m/s, `DM_DIR` · `WHEEL_DIR` · IMU 피치 부호는 실측해서 `config.h` 한 곳에만
- 모터 범위 — PMAX · VMAX · TMAX 는 모터 레지스터 = `config.h`

## 3. 다음 단계

| # | 할 일 | 코드 | 통과 기준 |
|---:|---|---|---|
| 1 | ~~좌 · 우 DM ID 설정 (한 개씩)~~ **완료** | `dm_config_tool` `L` / `R` | 재부팅 후 `s` → `2 motors`, ID 1 / 0x11 · 2 / 0x12, P/V/T MAX `OK` |
| 2 | ~~DM 2개 동시 구동~~ **완료** | `dm_dual_test` | 양쪽 `+` 에 두 다리 같은 방향, 한 쪽 고장 시 양쪽 정지 |
| 2a | 고관절 토크 실측 · 중력보상 근거 | `hip_step_test` v2 | 바퀴 띄운 무부하 `g` · `n` 스윕 플롯, 들기 모드로 **토르소가 뜨기 시작하는 ff** 기록 → 통합 펌웨어 `FF_SCALE` 근거 |
| 2b | 안전 보강 | `dm_config_tool` `t`, 기구 | DM Timeout 100 ms (현재 0 = 꺼짐), 기계식 스토퍼 펼침 −22° / 접힘 +22 ~ 25° |
| 3 | 좌우 바퀴 속도 제어 · 전진 방향 부호 | odrivetool | 두 바퀴 같은 속도 명령에 같은 방향으로 전진 (완료: M0 · M1 캘리브) |
| 4 | ODrive 를 CAN2 로 전환 | `host/odrive_can_setup` → `odrive_can_test` | 두 휠 0.5 turn/s, 엔코더 2 ms 수신 |
| 5 | 통합 펌웨어 — 모터 4개 + IMU | `twlr_teensy` (01 §7) | loop < 2.2 ms, 피드백 끊김 감시 DM 30 ms · ODrive 300 ms · IMU 50 ms |
| 6 | 한 글자 명령 → USB 패킷 | `bridge.*` · `protocol.py` | C++ · Python 같은 바이트(교차 테스트), CRC 틀린 패킷 버림, 250 ms 끊기면 제자리 정지 |
| 7 | ROS 2 브리지 + 원격 조종 | `twlr_bridge` · `teleop` | 아래 §4 |
| 8 | 시험 지그 1 → 2 → 3 단계 | — | 9주차 발표 T1 ~ T7 |

한 글자 명령(`e` · `h` · `+` …)은 지우지 않고 `#define DEBUG_CLI` 로 켜고 끌 수 있게 남깁니다. 패킷 모드에서도 단품 점검을 계속 할 수 있게 하기 위해서입니다.

## 4. 원격 조종 — 노트북 vs 게임패드

| 방식 | 연결 | 장점 | 단점 | 쓸 때 |
|---|---|---|---|---|
| 노트북 키보드 (SSH 안에서 `teleop_twist_keyboard`) | 노트북 → Wi-Fi → Khadas | 장비 추가 없음, 지금 바로 가능 | 키 입력이 켜짐/꺼짐뿐, 터미널 창이 선택돼 있어야 함, Wi-Fi 끊기면 조종 불가 | 지그 시험 · 저속 점검 |
| 노트북에서 ROS 2 직접 (`ROS_DOMAIN_ID` 같게) | 노트북 Ubuntu + Jazzy ↔ Khadas, 같은 공유기 | rviz · plotjuggler 로 실시간 모니터링 | 같은 네트워크 설정 필요, 끊김 위험은 같음 | 모니터링 · 튜닝 |
| **게임패드를 로봇에 직접** (`joy` + `teleop_twist_joy` 를 Khadas 에서 실행) | 패드 → 2.4 GHz 수신기(USB) 또는 블루투스 → Khadas | 아날로그 스틱으로 부드러운 속도 명령, **누르고 있어야만 움직이는 버튼**(deadman), Wi-Fi 가 끊겨도 조종 유지 | 패드 · 수신기 구매, 블루투스는 페어링 · 재연결이 번거로울 수 있음 | **바닥 주행 · 밸런싱** |

권장

- **지금 ~ 지그 시험**: 노트북 SSH + 키보드. 추가 장비 없이 됩니다.
- **바닥 주행부터**: 게임패드를 **로봇(Khadas)에 직접** 붙이고, 노트북은 모니터링만. 조종 경로에서 Wi-Fi 를 빼는 것이 핵심입니다.
- 연결은 **USB 수신기형**(2.4 GHz 동글)이 리눅스에서 가장 덜 번거롭습니다. 블루투스로 하려면 먼저 Khadas 에 블루투스가 잡히는지 `bluetoothctl show` 로 확인합니다.
- `teleop_twist_joy` 는 `require_enable_button: true` 로 두어 버튼을 떼면 `/cmd_vel` 이 0 이 되게 합니다.

```bash
sudo apt install ros-jazzy-joy ros-jazzy-teleop-twist-joy
ros2 run joy joy_node                                  # /joy
ros2 launch teleop_twist_joy teleop-launch.py joy_config:=xbox   # /cmd_vel, 패드 종류에 맞게
```

Teensy 에 블루투스 모듈을 직접 다는 방식은 쓰지 않습니다. ROS 2 와 같은 명령 경로가 두 개 생겨 안전 로직이 갈립니다.

### 끊겼을 때 (01 · 02 의 3중 워치독)

| 끊긴 것 | 동작 |
|---|---|
| 조종 입력 (`/cmd_vel` 1 s 없음) | 속도 0, 제자리 균형 유지 |
| ROS 2 ↔ Teensy (패킷 250 ms 없음) | 속도 0, 제자리 균형 유지 (enable · balance 는 마지막 값 유지) |
| Teensy ↔ 모터 | DM 은 CAN Timeout 100 ms 로 스스로 정지, ODrive 는 워치독 0.2 s 로 IDLE |
| 모든 경우 | 하드웨어 비상 스위치(BOM 추가 품목)가 최종 |

## 5. 2026-11-16 시험 요약

자세한 절차는 [05 고관절 구동 시험 절차](05-hip-drive-test-procedure.md), 토크 해석은 [06](06-hip-torque-gravity-comp.md).

- `dm_config_tool` 스캔: L `0x01 / 0x11`, R `0x02 / 0x12`, 둘 다 MIT · PMAX 12.5 / VMAX 10 / TMAX 28 · Timeout 0(꺼짐)
- `dm_dual_test` 유지 게인 Kp 5 로는 다리 무게에 밀려 `+` 를 눌러도 거의 안 움직임 → Kp 30 의 `hip_step_test` 로 전환
- `h` 유지 시 몸무게로 양쪽 +5 ~ 7° 처짐, 토크 −2.8 ~ −3.4 N·m — 양쪽 부호가 같아 `dir` 확인
- 바퀴 접지 · 토르소 고정 상태에서 `n`(펼침) 이 −0.9° 에서 막힘 → 추종 오차 10° 에서 양쪽 자동 정지 (보호 로직 의도대로 동작)
- v2 들기 모드(Kp 80 · ff) + 토르소 고정 해제(사람은 앞뒤 기울어짐만 잡음)로 **바퀴 접지 상태에서 접힘 · 펼침 성공** (영상 2편, 11주차 추가분 발표)
- 정지 중 속도 ±0.1 °/s 깜빡임은 MIT 피드백 12비트 양자화(칸 ≈ 0.28 °/s, 정확히 0 인 칸 없음) — 잡음으로 봄

## 6. 원격 조종 · failsafe 보강 (2026-11-16 검토)

- 블루투스 패드는 끊길 수 있다는 전제로 설계: **밸런싱은 Teensy 안에서 무선과 무관하게** 돌고, 조종기는 목표값(v, ω, 높이)만 보냄
- 명령 끊김 0.2 ~ 0.3 s → 목표 속도를 천천히 0 (밸런싱 유지, 모터 끄지 않음), 1 ~ 2 s → 낮은 자세로 정지, 피치 35 ~ 40° 초과 → 토크 차단
- 데드맨 버튼, 입력 기울기 제한(재연결 순간 튐 방지), 무선과 무관한 물리 비상정지
- 후보: 블루투스 패드(외장 동글 + Wi-Fi 5 GHz), 2.4 GHz 동글 패드(F710 등), RC 송수신기 → Teensy SBUS/CRSF 직결(수신기 자체 failsafe, Khadas 와 무관)
- 스틱으로 토크를 직접 주지 않음 — 스틱 → 정규화 · 데드존 → 목표값 → 기울기 제한 → 제어기 → 토크
- 시험: 지그 위에서 바퀴를 굴리는 중 조종기 전원을 일부러 끄고 타임아웃이 실제로 걸리는지 확인 (리눅스 joy 는 마지막 값을 계속 낼 수 있음)
