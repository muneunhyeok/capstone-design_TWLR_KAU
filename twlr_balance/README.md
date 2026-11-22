# TWLR 밸런싱 코드

## ★ v3 (2026-11-22): 밸런싱을 Teensy 로 옮김 — 이제 기본은 `main_teensy.py`
```
[Teensy 4.1]  twlr_bridge.ino v3   CAN1(1 Mbps) 하나에 힙 DM x2 + ODrive(노드 3=왼쪽 axis0, 4=오른쪽 axis1)
   500 Hz: 힙 홀드 + IMU   /   250 Hz: 밸런싱 계산 + ODrive 속도명령(CAN) + 엔코더 요청(RTR) + 안전
     │ USB 시리얼 (250 Hz 상태, 명령)
[Khadas]  main_teensy.py   키보드 → 명령/게인, 상태줄, CSV 로그 (~/logs/tbal_*.csv)
```
- 제어식은 파이썬 버전과 같음: `th_ref = -(KPv*(v-v_ref) + KIv*∫(v-v_ref))`, `a = KPth*(thCoM - th_ref) + KDth*wCoM`, `v_cmd += a*dt`
  (기본 게인 = LQR 과 같은 값. config.py 의 `PID_*` 를 시작할 때/`b` 누를 때 Teensy 로 보냄)
- 안전 (Teensy 안에서, 걸리면 바퀴 IDLE): 넘어짐 30° / IMU 50 ms / 힙 fault·홀드 아님 / ODrive 하트비트·엔코더 끊김·에러 /
  closed-loop 진입 실패 / **PC(파이썬) 300 ms 끊김** / 시작 시 8° 초과. 힙 fault 면 힙 끄고 바퀴도 정지.
- 펌웨어 v3 는 'B' 를 안 보내면 v2 처럼 동작 → 예전 `main.py`(USB 바퀴) 도 그대로 쓸 수 있음 (비교/백업용).
- ODrive 준비: CAN 1 Mbps, 노드 3/4 (odrivetool 에서 설정·저장 완료), 부팅 때 엔코더 오프셋 자동 캘리브레이션(바퀴 띄우고 전원).
- **ODrive 워치독은 꺼져 있음**: Teensy 가 멈추면(펌웨어 크래시) 바퀴가 마지막 속도로 계속 돌 수 있음 → `x` 와 배터리 분리가 최후 수단.

### 실행 순서 (Teensy 버전)
```bash
cd ~/twlr_balance/firmware/twlr_bridge
arduino-cli compile --fqbn teensy:avr:teensy41 --output-dir ~/arduino/build/twlr_bridge .
teensy_loader_cli --mcu=TEENSY41 -w -v ~/arduino/build/twlr_bridge/twlr_bridge.ino.hex
cd ~/twlr_balance && source ~/odrive-env/bin/activate
python3 tools/check_sensors.py --no-odrive      # [CAN ODrive] 살아있음 L=1 R=1, 바퀴 굴리면 속도 +, th_body 부호
python3 main_teensy.py
```
1. 거치대 위: `e` → 다리 시작 자세 → `h` (상태줄 `hip=7`, `od=LR`)
2. 바닥에 내려놓고 손으로 균형점 → `c` (thCoM ≈ 0)
3. `b` → 손 천천히 뗌. 상태줄에 `(CL)` 붙으면 바퀴 closed-loop
4. 튜닝: 빠르게 덜덜 → `-`, 천천히 쓰러짐 → `=`, 한쪽으로 감 → `[` `]`. 세부는 `1~4` 고르고 `,` `.`
5. 끝: `b` (밸런싱만) / `x` (전부) / `q`

검증: 펌웨어 코드를 PC 에서 가짜 Arduino/CAN/IMU 로 컴파일해 역진자 플랜트와 폐루프로 돌려봄 → 균형 유지(±0.5°),
0.2 m/s 추종, PC 끊김·넘어짐 안전 동작 확인. 실제 Teensy 컴파일·실물은 아직.

---

## (이전) v2 — 파이썬 3-프로세스 + Teensy 브리지 (main.py)

## v2 변경점 (2026-11-22)
| 문제 | 증상 | 고친 것 |
|---|---|---|
| 펌웨어가 float 15개(진단 3개 추가)를 보내는데 파이썬은 12개를 기대 | 패킷을 하나도 못 읽음 → 계속 "IMU 끊김" | `teensy_link.py` `RX_FMT = "<I15fBB"`, 진단값 공유메모리에 추가 |
| 펌웨어에서 roll/pitch **각도만** 바꾸고 자이로는 그대로 | pitch 각속도로 roll 축(gy)을 쓰게 됨 → D항이 엉뚱 → 절대 안 섬 | 펌웨어가 `gy = -센서gx`(pitch율), `gx = 센서gy`(roll율)로 보냄. **pitch 와 gy 는 이제 항상 짝** |
| 파이썬이 안 읽으면 Teensy `Serial.write` 가 멈출 수 있음 | CAN 루프 밀림 → 힙 fault | 송신 버퍼 자리 없으면 그 패킷 버림 |
| IMU 초기화 실패해도 계속 poll | 멈출 위험 | `imuFound` 플래그 |
| 적분에 고정 dt(10 ms) 사용 | ODrive USB 가 느려 주기가 밀리면 게인이 줄어든 것처럼 됨 | 실제 루프 시간으로 적분 |
| 매 주기 ODrive USB 호출 8번 | 100 Hz 못 맞출 수 있음 | 워치독 feed 5주기마다, 위치는 속도 적분 → 4번 |
| 없는 워치독 속성에 대입해도 에러가 안 남 | 나중에 feed 에서 제어 프로세스가 죽을 수 있음 | `hasattr` 로 진짜 있는지 확인 |
| `check_sensors.py` | 축/부호를 눈으로만 봐야 함 | Teensy 루프 Hz·IMU 진단 표시 + **"자이로-각도 짝 확인 %"** 자동 계산 |

**⚠ 펌웨어와 파이썬은 반드시 같이 올릴 것** (옛 펌웨어 + 새 파이썬, 새 펌웨어 + 옛 파이썬 둘 다 안 됨).

### 힙 처짐 여유 (밸런싱 중 힙 fault 가 나면 여기부터)
- 서 있을 때 다리 하나 토크 ≈ 6.3 N·m (`model.hip_torque_nm(0)`). kp 60 이면 처짐 ≈ 6.3/60 rad ≈ **6°**.
- 펌웨어 fault 기준은 10° / 10 N·m → 바닥에 내려놓는 순간 여유가 약 4° 밖에 안 남음.
- 밸런싱 중 `hip=` 값이 7 에서 떨어지거나 힙이 꺼지면: `config.HIP_KP` 를 90~100 으로 (처짐 ~4°) 하거나, Δφ 부호 확인 후 `TFF_ENABLE = True`.

전제: 다리는 **h 를 누른 자세(시작 자세)에 고정**하고, 바퀴로만 균형을 잡는다.

```
[Teensy 4.1]  twlr_bridge.ino  ── 500 Hz: 힙 DM4340P x2 홀드(CAN1) + BNO085 읽기
     │ USB 시리얼 (250 Hz 바이너리)
[Khadas]
  P2 teensy_link.py   시리얼 받기/보내기 → 공유메모리
  P1 control_task.py  ODrive(USB) + CoM 추정 + 밸런스 (100 Hz, 혼자 도는 프로세스)
  P3 main.py          키보드 + CSV 로그 + 상태줄
```
- 프로세스마다 GIL 이 따로라서 로그/화면이 제어를 막지 않음. 공유는 `shared.py`(최신값 배열) + 큐.
- 제어식: `th_CoM = th_IMU + th_offset(Δφ) + trim`,  `a = KX·x + KV·v + KTH·th_CoM + KW·w_CoM`,  `v_cmd += a·dt` → ODrive 속도명령
- 제어 방식은 **LQR / 두 겹 PID** 중 선택 (`config.CTRL_MODE`, 실행 중 `p` 키). PID 기본 게인은 LQR 과 똑같이 동작하는 값이라 같은 출발점에서 손튜닝 가능.
- `th_offset(Δφ)`, 중력보상 토크는 `model.py` (소령 MATLAB 정역학 + CAD 질량 추정). 기본 게인은 LQR(`tools/lqr_gains.py`).

## 파일
| 파일 | 역할 |
|---|---|
| `config.py` | **모든 설정·부호·게인 (여기만 고침)** |
| `model.py` | CoM 오프셋, 중력보상 토크 근사식 |
| `shared.py` | 공유메모리 필드 이름, 큐 |
| `main.py` | 실행(P3), 키보드/로그/상태 |
| `control_task.py` | P1 제어 루프 + 안전 |
| `teensy_link.py` | P2 Teensy 프로토콜 |
| `odrive_io.py` | 바퀴(실제 / 시뮬) |
| `sim.py` | `--sim` 용 역진자 플랜트 |
| `firmware/twlr_bridge/twlr_bridge.ino` | Teensy 펌웨어 (hip_step_test 기반) |
| `tools/check_sensors.py` | **센서 값·부호 확인 (바퀴 안 켬)** |
| `tools/sim_test.py`, `tools/lqr_gains.py` | 시뮬 자동 테스트, 게인 재계산 |

## 0. 먼저 시뮬로 돌려보기 (하드웨어 없이)
```bash
python3 main.py --sim        # b: 밸런싱 시작, i/k: 전진/후진, x: 정지, q: 종료
python3 tools/sim_test.py    # 자동 테스트 (균형 유지, 0.2 m/s 추종, 루프 dt)
```

## 1. Khadas 설치 (한 번)
```bash
scp -r twlr_balance khadas@192.168.0.14:~/          # 노트북 PowerShell 탭에서
# Khadas 탭:
source ~/odrive-env/bin/activate                     # odrivetool 깔아둔 venv 재사용
pip install pyserial
arduino-cli lib install "Adafruit BNO08x"
cd ~/twlr_balance/firmware/twlr_bridge
arduino-cli compile --fqbn teensy:avr:teensy41 --output-dir ~/arduino/build/twlr_bridge .
teensy_loader_cli --mcu=TEENSY41 -w -v ~/arduino/build/twlr_bridge/twlr_bridge.ino.hex
```
※ picocom / odrivetool 이 켜져 있으면 끄고 실행 (포트·USB를 한 프로그램만 잡을 수 있음).

## 2. 센서 값 확인 (밸런싱 전에 꼭) — 바퀴는 안 켬
```bash
python3 tools/check_sensors.py              # Teensy(IMU+힙) + ODrive(바퀴)
python3 tools/check_sensors.py --no-odrive  # IMU/힙만 먼저 볼 때
```
- `[Teensy] 수신 ~250 Hz OK`, `IMU ok=1`, `fault=0`, `[ODrive] 전압 22~25 V`, `준비 True` 가 다 OK(초록)인지
- 아래 체크리스트를 이 화면 보면서 하나씩 확인 → `config.py` 수정 → 다시 실행해서 확인
- 로그: `~/logs/check_*.csv`

## 2-1. 부호 확인 체크리스트 (★ — 처음 한 번, 거치대 위)
| 항목 | 확인 방법 | 설정 |
|---|---|---|
| IMU 축 | 로봇을 앞뒤로 기울이며 로그의 `th_body` 가 변하는지 (v2 펌웨어면 pitch 가 변해야 정상) | `IMU_AXIS` |
| 자이로 짝 | 앞뒤로 빠르게 몇 번 흔들기 → `자이로-각도 짝 확인` 90% 이상 (낮으면 펌웨어가 v2 아님) | (펌웨어) |
| IMU 부호 | 몸통 윗부분을 **바퀴 + (전진) 방향**으로 기울였을 때 `th_body` 가 + (자이로 부호는 같이 따라감) | `IMU_SIGN` |
| IMU 영점 | 몸통 판 수평일 때 `th_body` 값 | `IMU_MOUNT_DEG` |
| CoM 방향 | CAD +y(무릎·몸체피벗 쪽)가 로봇 앞쪽인지 | `COM_OFF_SIGN` |
| Δφ 부호 | 손으로 다리를 **펴면** Δφ 가 + 되는지 (팀원 로그상 -1 추정) | `DPHI_SIGN` |
| 시작 자세 | h 자세에서 바퀴축이 힙축 아래 336 mm 면 0, 아니면 그 자세의 Δφ | `DPHI_AT_H` |
| 바퀴 방향 | (2026-11-22 확인) 앞으로 굴리면 ODrive 값 - → WHEEL_DIR = (-1, -1) | `WHEEL_DIR` |

## 3. 로봇에서 실행 순서
```bash
cd ~/twlr_balance && source ~/odrive-env/bin/activate
python3 main.py --calibrate      # ODrive 전원 껐다 켰으면 (바퀴 공중!). 아니면 --calibrate 빼기
```
1. **거치대 위**: `e`(힙 켜기) → 손으로 다리를 시작 자세에 → `h`(고정). 상태줄 `hip=7` 이면 OK
2. 사람이 몸통을 잡고 **바닥에 내려놓기** (끈으로 위에서 잡아주면 더 좋음)
3. 손으로 균형 잡히는 각도 찾고 그 상태로 `c` → trim 자동 설정 (상태줄 thCoM ≈ 0)
4. `b` → 밸런싱 시작. 손을 천천히 뗌.  흔들리면 `-`(게인↓), 굴러가면 `[` `]`(trim)
5. 끝: `b`(밸런싱만 정지) 또는 **`x`(전체 정지)**, `q` 종료. 로그: `~/logs/bal_*.csv`

## PID 모드로 튜닝하기
```
목표속도 → [바깥: 속도 PI] → 목표 기울기 th_ref → [안쪽: 기울기 PD] → 바퀴 가속도 → 속도명령
```
| 키 | 게인 | 기본값 | 의미 |
|---|---|---|---|
| `1` | KP_th | 27.9 | 기울기 P: 기운 만큼 바퀴로 받치기 (작으면 넘어짐, 크면 덜덜) |
| `2` | KD_th | 4.65 | 기울기 D: 흔들림 잡기 |
| `3` | KP_v  | 0.078 | 속도 P: 굴러가면 목표 기울기를 반대로 (도망 방지) |
| `4` | KI_v  | 0.036 | 속도 I: 천천히 밀려나는 것 잡기 (= 위치 유지) |

`1~4`로 고르고 `,` `.`로 ×0.9 / ×1.1. 상태줄에 `>` 표시된 게 선택된 게인.
튜닝 순서: (3,4 를 작게) → 1 올려서 버티게 → 덜덜 떨면 2 → 3 으로 굴러가는 것 멈추기 → 4 로 드리프트 잡기.
실행 중 바꾼 값은 종료하면 사라지니까, 좋은 값을 찾으면 `config.py` 의 `PID_*` 에 적어둘 것.

## 안전장치 (자동으로 바퀴 정지)
기울기 30° 초과 / IMU 50 ms 끊김 / 힙 고장·홀드 아님 / ODrive 에러 / 시작 시 기울기 8° 초과.
ODrive 워치독 0.2 s: 파이썬이 죽으면 바퀴가 스스로 멈춤. Teensy: PC 끊기면 중력보상만 끄고 다리는 홀드 유지.

## 알아둘 것 / 한계
- **dt_max**: 상태줄에 표시. 100 Hz(10 ms) 목표에서 20 ms 넘게 자주 튀면 `CONTROL_CPU` 지정, `sudo chrt -f 80 python3 main.py`, 그래도 안 되면 밸런스 루프를 Teensy로 옮길 것 (ODrive 를 Teensy CAN 에 연결).
- **중력보상(`TFF_ENABLE`)은 기본 꺼짐.** 거치대 위에선 몸무게가 안 실려서 켜면 다리가 튀어나감. 바닥에서 Δφ 부호 확인 후 켜기.
- `KX`(위치 유지)를 0 으로 하면 trim 1° 오차당 약 0.3 m/s 로 계속 굴러감 → 기본 1.0.
- 확인 안 된 것: Teensy 실제 컴파일(문법만 확인), odrive 0.5.1 워치독 속성 이름(없으면 자동으로 워치독 없이 동작), BNO085 장착 축.
