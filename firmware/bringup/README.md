# 단품 시험 (bring-up)

통합 펌웨어 전에 모터 1개를 CAN 으로 구동해 보는 시험 스케치입니다.

| 폴더 | 내용 | 상태 |
|---|---|---|
| [`dm_single_test/`](dm_single_test/) | Teensy 4.1 CAN1(22=CTX1, 23=CRX1, 1 Mbps) → SN65HVD230 → DM-J4340P-2EC 1개, MIT 모드. 시리얼 명령으로 활성화 · 정속 회전 · 정지 | 업로드·시리얼 명령 확인, **모터 피드백 수신은 미확인** |

## 시리얼 명령 (115200)

| 키 | 동작 |
|---|---|
| `e` | 활성화 (자유 모드, 토크 0) |
| `h` | 현재 위치 유지 (`e` 후 피드백이 와야 동작) |
| `a` / `b` | 정방향 / 역방향 회전 — 가속 → 정속 5 s → 감속 후 정지 |
| `s` | 회전 중 감속 정지 |
| `d` | 비활성화 |

회전 설정은 파일 위쪽 상수로 조절합니다.

| 상수 | 기본값 | 의미 |
|---|---|---|
| `SPIN_SPEED` | 4.0 rad/s (≈ 38 rpm) | 목표 속도. 24 V 최대 무부하 52 rpm(5.45 rad/s) 이하로 설정 |
| `SPIN_HOLD_S` | 5.0 s | 정속 구간 시간 |
| `SPIN_ACCEL` | 2.0 rad/s² | 가속 · 감속률 |
| `SPIN_KD` | 1.0 | 속도 제어 이득. 걸리는 토크 = `SPIN_KD × (목표 속도 − 현재 속도)`, 최대 ≈ `SPIN_KD × SPIN_SPEED`. 0 이면 안 됨 |
| `MAX_SPIN_TORQUE` | 6.0 N·m | 측정 토크가 이 값을 넘으면 자동 비활성화 (정격 9 N·m). `SPIN_KD` 를 올릴 때 같이 올릴 것 |

안전 로직: 피드백 끊김(활성화 직후 1 s, 이후 200 ms) · 모터 오류 코드 ≥ 8 · 로터 온도 60 °C · 위치 오차 1 rad(유지 상태) · 회전 중 토크 초과 시 `DISABLED` 로 꺼집니다.

## Khadas VIM4 (aarch64, Ubuntu 24.04)에서 올리는 방법

Arduino IDE 는 x86 용이라 aarch64 에서 쓸 수 없어 `arduino-cli` + `teensy_loader_cli` 를 사용했습니다.

```bash
export PATH=$PATH:~/arduino/bin
arduino-cli core install teensy:avr           # 1회 (teensy:avr 1.62.0 확인)
arduino-cli compile --fqbn teensy:avr:teensy41 --output-dir build dm_single_test
teensy_loader_cli --mcu=TEENSY41 -w -v build/dm_single_test.ino.hex   # PROGRAM 버튼을 눌러 업로드
screen /dev/ttyACM0 115200                    # 종료: Ctrl+A, K, y
```

- `/dev/ttyACM0` 접근에는 `dialout` 그룹과 Teensy udev 규칙(`00-teensy.rules`)이 필요합니다.
- 명령(`e` 등)은 셸이 아니라 `screen` 안에서 입력해야 합니다.
- 업로드 후 Teensy 가 HalfKay 부트로더(USB ID `16c0:0478`)에 머물면 `/dev/ttyACM*` 이 생기지 않습니다. `lsusb | grep 16c0` 으로 확인하고 다시 업로드합니다 (시리얼은 `16c0:0483`).

## 현재 상태 · 미해결 (2026-11-06)

- `e` 전송 후 `CAN frames received = 0` → `DISABLED: no feedback` 이 나옵니다. 모터 응답 프레임이 한 번도 들어오지 않는 상태입니다.
- 모터는 전원을 켜자마자 LED 가 초록입니다 (매뉴얼 기준 초록 = 활성화).
- 점검 순서
  1. `d` 입력 시 LED 가 빨강으로 바뀌는지 — 바뀌면 Teensy → 모터 방향 · ID 1 은 정상이고 수신(RXD · 트랜시버) 쪽 문제
  2. 트랜시버 배선: TXD ← Teensy 22, RXD → Teensy 23, VCC 3.3 V, Rs → GND, 세 장치 GND 공통
  3. CAN H/L 방향 (XT30 2+2 작은 핀: 위 CAN_L, 아래 CAN_H)
  4. 전원 OFF 상태에서 CANH–CANL 저항 ≈ 60 Ω (종단)
  5. USB2CAN + DM Debug Tool(CAN 탭)로 같은 모터를 확인해 모터 · ID 문제와 Teensy 쪽 문제를 분리
- 모터 전원 투입 시 Teensy USB 가 끊기는 현상이 있었습니다 (노이즈 · 접지 의심). 모터 전원 OFF 에서 연결 → 몇 분 관찰 → 전원 ON 순으로 상관을 확인할 것.

## 확인이 필요한 가정 (매뉴얼에 없는 값)

- `P_MAX 12.5 / V_MAX 10 / T_MAX 28` 은 모터 내부 설정과 같아야 합니다. `a` 구동 후 실제 회전량으로 검증할 것.
- 활성화 `FF…FC` · 비활성화 `FF…FD` · 영점 `FF…FE` 특수 프레임은 매뉴얼(MIT 모드 프레임 형식)에 명시되지 않아 알려진 Damiao 프로토콜 기준으로 작성했습니다. `0xFE` 는 정속 회전이 끝난 뒤 위치 범위(±12.5 rad)를 넘은 값을 0 으로 되돌리는 데 씁니다.
- 모터 CAN ID 1 · Master ID(피드백 ID)는 기본값 가정이며 디버그 툴로 확인이 필요합니다.
