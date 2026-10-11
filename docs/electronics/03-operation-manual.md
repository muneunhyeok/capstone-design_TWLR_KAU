# 03 — 운용 방법 (1회 설정 · 켜기 · 밸런싱 · 주행 · 끄기 · 비상 대처 · 튜닝)

> **전제**: [01 §12](01-can-integration-plan.md#12-안전--추가-구매)의 BOM 추가 부품(퓨즈 · 비상 스위치 · 종단저항 · 트위스트 페어 · 브레이크 저항)을 모두 단 상태
> 출처: [2학기 통합본 발표](../presentations/2026-2-integrated/) PART 08 (슬라이드 99–109)
> 앞 문서: [01 CAN 통합 구동 계획](01-can-integration-plan.md) · [02 코드 해설 · 원리](02-control-principles.md)

**원칙** — 로봇이 넘어지는 것보다 사람이 다치는 것이 더 큰 사고. **망설이면 비상 스위치.**

## 목차

1. [최종 구성 · 준비물](#1-최종-구성--준비물)
2. [처음 한 번만 하는 설정](#2-처음-한-번만-하는-설정)
3. [켜기 · 밸런싱 · 주행 · 끄기](#3-켜기--밸런싱--주행--끄기)
4. [비상 대처 · 고장 코드](#4-비상-대처--고장-코드)
5. [게인 튜닝](#5-게인-튜닝)
6. [배터리 · 점검](#6-배터리--점검)

---

## 1. 최종 구성 · 준비물

```
전원  6S 22.2 V → 퓨즈 30 A(추가) → 메인/비상 SW(추가) → WAGO 분배 → DM · ODrive · 5 V
신호  LattePanda ─USB─ Teensy 4.1 ─CAN1 · CAN2 (트위스트 + 120 Ω)─ DM × 2 · ODrive
```

| 추가 부품 | 위치 | 운용에서의 역할 |
|---|---|---|
| 30 A 퓨즈 | 배터리 + 직후 | 단락 · 과전류 시 차단 |
| 메인 · 비상 스위치 | 퓨즈 다음 · 손 닿는 곳 | 비상 시 전체 전원 차단 (1순위) |
| 120 Ω (CAN1 끝) | DM R 쪽 끝 | 반사 방지 — 양 끝 합쳐 60 Ω |
| 트위스트 페어 | CAN1 · CAN2 | 모터 잡음 상쇄 |
| 2 Ω 브레이크 저항 | ODrive | 감속 회생 전압 흡수 (DM 32 V 보호) |

**준비물** — 노트북(SSH 또는 모니터 · 키보드) · 충전된 배터리(≥ 24.5 V) + 예비 · LiPo 방화백 · 멀티미터 ·
서포터(바퀴를 띄우는 받침) · USB-TTL 3.3 V(DM 설정 때만) · 주행 공간 3 × 2 m 이상 · **2인 1조 — 조작 1 · 안전 1**

## 2. 처음 한 번만 하는 설정

순서대로, 각 단계의 확인 항목을 통과하면 다음으로.

| # | 대상 | 어디서 | 내용 | 확인 |
|---:|---|---|---|---|
| 1 | DM4340P | Windows · Damiao 툴 | 모터 1개씩(24 V · 전류제한 3 A), ESC_ID 1/2 · MST_ID 0x11/0x12 · MIT · 12.5/8/28 · Timeout 2000, 기준 자세에서 영점 — [`host/dm4340p_setup.md`](../../host/dm4340p_setup.md) | `dm_single_test` → `e` → state 1 · 각도 0 근처 |
| 2 | ODrive | LattePanda · odrivetool | [`host/odrive_can_setup.odrivetool.py`](../../host/odrive_can_setup.odrivetool.py) 블록 단위 붙여넣기 → 재부팅 → 휠 공중에서 모터 캘리브 1회 · `pre_calibrated = True` | `dump_errors(odrv0)` 에러 없음 · node_id 0 / 1 |
| 3 | Teensy 펌웨어 | 노트북 · Arduino IDE 2 | Teensyduino(FlexCAN_T4 포함) · 라이브러리 Adafruit BNO08x · 보드 Teensy 4.1 / USB Type Serial / 600 MHz → `twlr_teensy.ino` 업로드 | — |
| 4 | ROS 2 패키지 | LattePanda · 터미널 | 아래 명령 | `ls -l /dev/twlr_teensy` · `ros2 pkg list \| grep twlr` |

```bash
sudo apt install ros-jazzy-teleop-twist-keyboard python3-serial
mkdir -p ~/twlr_ws/src && cp -r twlr_bringup ~/twlr_ws/src/
cd ~/twlr_ws && colcon build --symlink-install
echo "source ~/twlr_ws/install/setup.bash" >> ~/.bashrc
sudo cp src/twlr_bringup/config/99-twlr-teensy.rules /etc/udev/rules.d/   # Teensy 이름 고정
sudo udevadm control --reload && sudo udevadm trigger
```

### 방향 · 부호 · 균형각 측정 (`config.h` 의 [측정] 값)

| config.h 값 | 어떻게 재나 | 넣는 값 |
|---|---|---|
| `DM_DIR_L / R` | `dm_single_test` · `p` → 다리를 손으로 펴 봄 | 펼 때 pos 증가 → +1 |
| `WHEEL_DIR_L / R` | `odrive_can_test` · `c` → `l` → `v` | 전진 방향으로 돌면 +1 |
| IMU 피치 축 | READY 에서 `/twlr/pitch` 보며 앞으로 기울임 | 안 변하면 `EULER_ROLL = true` |
| `IMU_PITCH_SIGN` | 앞으로 기울였을 때 | pitch 증가 → +1 |
| `THETA_EQ_RAD` | 균형점을 손으로 잡고 5 s 평균 | 보통 −0.1 ~ +0.1 rad |
| DM 영점 | 기준 자세에서 `z` 키 | 재부팅 후에도 0 근처 |

```bash
# θ_eq 측정 (100 Hz × 5 s 평균) → 예: 0.034 → config.h: constexpr float THETA_EQ_RAD = 0.034f;
ros2 topic echo /twlr/pitch --field data | grep -v -- --- | head -n 500 | awk '{s+=$1} END {print s/NR}'
```

- **부호가 틀리면 밸런싱 시작 즉시 한쪽으로 튀어 나간다** → 반드시 서포터 위에서 먼저 확인. 값을 바꾸면 업로드(≈20 s) 후 재확인
- θ_eq 잡는 법: READY 상태에서 몸체 손잡이를 잡고 바퀴만 바닥에 살짝 닿게, 앞뒤 어느 쪽으로도 손에 힘이 안 걸리는 각도 유지. 밸런싱 후 한쪽으로 계속 흐르면 ±0.01 rad 씩 보정

## 3. 켜기 · 밸런싱 · 주행 · 끄기

### 전원 켜기 → READY (서포터 위, 바퀴가 공중에 뜬 상태)

`BOOT` LED 점등 · 연결 대기 → `IDLE` 힘 없음 · 각도만 → `CALIB` 바퀴가 천천히 앞뒤로 ≈3 s → `READY` 다리가 455 mm 로 천천히 이동

1. 로봇을 서포터에 올림 — 바퀴가 바닥 · 케이블에 닿지 않는지 확인
2. 배터리 연결 → 메인 스위치 ON — DM · ODrive LED 확인, **비상 스위치 위치 확인**
3. LattePanda 터미널 1: launch → 로그에 `serial connected` · `mode: IDLE`
4. 터미널 2: enable → 로그 `CALIB → READY` (**바퀴가 돎 — 손 · 케이블 주의**)
5. `faults` 가 비어 있는지 확인, 문제 있으면 [고장 코드 표](#고장-코드)

```bash
ros2 launch twlr_bringup twlr_bringup.launch.py                         # 터미널 1 — 로그를 계속 봄
ros2 topic pub --once /twlr/enable std_msgs/msg/Bool "{data: true}"      # 터미널 2 — 휠 캘리브 → READY
ros2 topic echo --once /twlr/faults
ros2 topic echo --once /twlr/battery --field voltage
```

### 밸런싱 시작 · 주행 (2인)

1. 한 명이 몸체를 잡고 서포터에서 내려 바퀴를 바닥에 놓음 (READY 유지)
2. 몸체를 균형각 근처로 세워 잡음 — θ_eq ± 10° 밖이면 BALANCE 진입 거부
3. 다른 한 명이 `balance true` → 바퀴가 버티는 힘이 느껴지면 손을 천천히 놓음
4. 10 s 제자리 확인 → teleop 으로 주행

```bash
ros2 topic pub --once /twlr/balance std_msgs/msg/Bool "{data: true}"
ros2 run teleop_twist_keyboard teleop_twist_keyboard                    # 새 터미널, 창이 선택돼 있어야 키가 먹힘
ros2 topic pub --once /twlr/height std_msgs/msg/Float32 "{data: 400.0}" # 높이 [mm]
ros2 topic echo /twlr/hip_temp ; ros2 topic hz /imu/data ; ros2 topic echo /twlr/loop_us   # 감시 (≈100 Hz, < 2200 µs)
```

| teleop 키 | | | |
|---|---|---|---|
| `u` 전진 좌 | `i` 전진 | `o` 전진 우 | `w` / `x` 직진 ±10 % |
| `j` 좌회전 | `k` **정지** | `l` 우회전 | `c` 회전 −10 % |
| `m` 후진 좌 | `,` 후진 | `.` 후진 우 | |

- 기본 0.5 m/s · 1.0 rad/s → **`x` 를 5번 눌러 ≈0.3 m/s 로 시작**. Teensy 가 0.6 m/s · 1.5 rad/s 에서 자름
- 키를 떼면 1 s 뒤 정지 · `k` = 즉시 정지
- 높이 290 – 520 mm, 약 20°/s 로 천천히. **452 mm 아래는 고관절 정격 초과 → 짧게만**. 코일 80 °C 초과 시 자동 510 mm

### 정지 · 종료 — 반드시 서포터 위에서 끈다

| # | 동작 | 결과 |
|---:|---|---|
| 1 | `k` 로 정지 → teleop Ctrl+C | |
| 2 | 들어서 서포터 위 | 바퀴 공중 |
| 3 | `balance false` | READY · 휠 토크 0 |
| 4 | `enable false` | IDLE · **다리 힘 풀림 → 서포터가 몸체를 받치는지 먼저 확인** |
| 5 | launch Ctrl+C → `sudo poweroff` | Ctrl+C 로 브리지를 끄면 enable false 가 자동 전송 |
| 6 | 스위치 OFF → 배터리 분리 · 보관 | 배터리는 맨 마지막 |

**하면 안 되는 것** — 바닥에서 `balance false`(휠 토크 0 → 그대로 넘어짐) · 바닥에서 `enable false` 나 launch Ctrl+C(다리 힘 풀림 + 휠 정지 → 주저앉으며 넘어짐) · 밸런싱 중 배터리 분리(넘어짐 + LattePanda 비정상 종료). **예외: 위험할 때는 넘어지더라도 비상 스위치.**

## 4. 비상 대처 · 고장 코드

| 상황 | 즉시 할 일 | 원인 · 다음 조치 |
|---|---|---|
| 앞뒤로 점점 커지는 흔들림 | 몸체를 잡고 서포터로 → balance false | Kθ · Kω 조정 · θ_eq 재측정 |
| 한쪽으로 튀어 나감 (폭주) | **비상 스위치 OFF** | WHEEL_DIR · IMU 부호 → 서포터에서 재확인 |
| 넘어짐 (35° 이상) | 자동 FAULT → 휠 정지 · 다리 1 s 댐핑 | 파손 확인 → enable false → true |
| LattePanda · 터미널 멈춤 | 로봇은 제자리 균형 유지 (v = 0) | 몸체를 잡고 서포터로 → 비상 스위치 |
| 고관절 과열 (HIP_TEMP) | 자동으로 510 mm 로 올라섬 | 주행 중단 · 서포터에서 식힌 뒤 재개 |
| 타는 냄새 · 연기 · 배터리 부풂 | 비상 스위치 → 배터리 분리 → 방화백 | 원인 확인 전 재가동 금지 |
| 배터리 21.0 V 이하 | 주행 중단 → 종료 순서 | 19.0 V 에서 ODrive 차단 → 휠 갑자기 정지 |

### 고장 코드

`/twlr/faults` 에 원인 이름이 쉼표로 찍힌다. **리셋**: 원인 제거 → `/twlr/enable false` → 1 s → `true`

| faults 이름 | 뜻 | 확인 · 조치 |
|---|---|---|
| `DM_L/R_TIMEOUT` | DM 피드백 30 ms 끊김 | CAN1 배선 · 120 Ω · DM 전원 · ID 1/2 · 0x11/0x12 |
| `DM_L/R_ERR` | DM state ≥ 8 | 전원 재투입 · 온도 · 디버깅 툴로 에러 확인 |
| `OD_L/R_TIMEOUT` | ODrive heartbeat 300 ms 끊김 | CAN2 배선 · baud 1 M · node 0/1 · DIP 120 Ω |
| `OD_L/R_ERR` | axis_error ≠ 0 | USB → odrivetool → `dump_errors(odrv0)` |
| `IMU` | BNO085 50 ms 응답 없음 | I²C 18/19 배선 · 3.3 V · 주소 0x4A |
| `FALL` | 기울기 35° 초과 | 넘어짐 — 파손 확인 후 리셋 |
| `CALIB` | 휠 엔코더 캘리브 실패 · 15 s 초과 | 바퀴가 공중인지 · 엔코더 커넥터 |
| `HIP_TEMP` | 코일 80 °C 초과 (경고, 동작 유지) | 자동 510 mm · 휴식 후 재개 |

DM state: `8` 과전압 · `9` 저전압 · `A` 과전류 · `B` MOS 과열 · `C` 코일 과열 · `D` 통신두절 · `E` 과부하

## 5. 게인 튜닝

순서가 중요하다 — θ_eq 가 맞아야 나머지 게인이 의미가 있다. **한 번에 하나 · 10–20 % 씩 · 바꿀 때마다 기록**, `config.h` 수정 → 업로드(≈20 s).

| # | 대상 | 시험 | 목표 |
|---:|---|---|---|
| 1 | θ_eq | 제자리 | 한쪽으로 흐르면 ±0.01 rad |
| 2 | Kθ · Kω | 손 놓고 서 있기 | 떨림 없이 10 s |
| 3 | Kv · Kx | 살짝 밀었다 놓기 | 출렁임 없이 복귀 |
| 4 | K_YAW | 제자리 회전 | 늦거나 흔들리지 않게 |
| 5 | 고관절 Kp · Kd · FF | 높이 바꾸기 | 처짐 · 떨림 없이 |

| 증상 | 조정 |
|---|---|
| 10 Hz 넘는 빠른 떨림 · 소리 | Kω ↓ (4.59 → 3.5) 또는 Kθ ↓ |
| 느리고 큰 앞뒤 흔들림 | Kθ ↑ · Kω ↑ |
| 한 방향으로 계속 흘러감 | θ_eq 재측정 · Kx ↑ |
| 멈출 때 앞뒤로 출렁 | Kv ↑ |
| 선회가 늦음 / 흔들림 | K_YAW ↑ / ↓ |
| 다리가 처짐 (높이 부족) | HIP_FF_SCALE 0.8 → 1.0 · HIP_KP ↑ |
| 다리가 덜덜 떨림 | HIP_KD_LOADED ↑ (3 → 4) |

```bash
ros2 bag record -o run01 /imu/data /joint_states /odom /twlr/pitch /cmd_vel
ros2 run plotjuggler plotjuggler        # sudo apt install ros-jazzy-plotjuggler-ros
```

## 6. 배터리 · 점검

| 전압 (6S) | 셀당 | 의미 |
|---|---|---|
| 25.2 V | 4.20 V | 만충 |
| 23.0 V | — | 운용 시작 권장 이상 |
| 22.8 V | 3.80 V | 보관 |
| 22.2 V | 3.70 V | 공칭 |
| **21.0 V** | 3.50 V | **운용 중단 권장** |
| 19.8 V | 3.30 V | 최저 |
| 19.0 V | — | ODrive 저전압 차단 (휠 갑자기 정지) |

**운용 전** — ☐ 배터리 ≥ 23.0 V ☐ 전원 OFF 에서 CANH–CANL ≈ 60 Ω ☐ 커넥터 · 나사 · 링크 핀 풀림 없음 ☐ 서포터 · 비상 스위치 손 닿는 곳 ☐ 주행 공간 3 × 2 m · 주변 사람 확인 ☐ 2인 1조

**운용 후** — ☐ `/twlr/hip_temp` 확인 후 보관 ☐ 보관 전압 22.8 V 로 충 · 방전 ☐ 방화백 보관 · 부풂 확인 ☐ rosbag 정리 · 튜닝값 기록
