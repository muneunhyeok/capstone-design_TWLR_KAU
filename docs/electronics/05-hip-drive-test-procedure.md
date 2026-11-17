# 05 고관절 DM4340P 구동 시험 절차 (Khadas · Teensy)

2026-11-16 · 노트북에서 SSH 로 Khadas 에 접속해 Teensy 스케치를 빌드 · 업로드하고, Teensy 가 CAN1 으로 고관절 DM4340P 두 개(L/R)를 동시에 구동한다.
ROS 없이 Teensy 단독 구동만 다루며, 2026-11-16 에 실제로 진행한 순서 그대로 정리했다. 스케치는 [`firmware/bringup/`](../../firmware/bringup/).

| 구간 | 장치 / 연결 | 비고 |
|---|---|---|
| 작업 PC | 노트북 (Windows PowerShell) | Khadas 와 같은 공유기 |
| 원격 접속 | SSH `khadas@<Khadas IP>` | 여러 노트북 동시 접속 가능 |
| 상위 보드 | Khadas VIM4 (Ubuntu 24.04, aarch64) | 빌드 · 업로드 · 시리얼 모니터 |
| 실시간 제어 | Teensy 4.1 (USB 시리얼 `/dev/ttyACM0`) | 500 Hz 루프 |
| 통신 | CAN1 1 Mbps, 핀 22 TX / 23 RX, SN65HVD230 | |
| 고관절 모터 | DM4340P L: CAN ID 1 / Master 0x11, R: 2 / 0x12 | MIT 모드 |

## 0단계 — 노트북 접속 준비

새 노트북은 SSH 만 되면 된다. 도구는 전부 Khadas 에 설치돼 있다.

1. 노트북을 Khadas 와 **같은 공유기**에 연결하고 PowerShell 에서 `ping <Khadas IP>` 로 응답을 확인한다.
2. `ssh khadas@<Khadas IP>` → 처음이면 `yes` → 비밀번호. 프롬프트가 `khadas@Khadas:~$` 가 되면 성공.
   - `ssh` 가 없다고 나오면 설정 → 선택적 기능 → OpenSSH 클라이언트 설치.
   - Khadas IP 를 모를 때는 Khadas 터미널에서 `hostname -I`.
3. 긴 작업은 `tmux new -s setup` 안에서 한다. 끊기면 다시 접속해 `tmux attach -t setup`.
4. tmux 를 켜면 `61;4;6;7;…c` 같은 글자가 입력줄에 생길 수 있다. 표시 문제일 뿐이다. `Ctrl+U` 로 지우고, 한 번만 `echo "set -sg escape-time 50" >> ~/.tmux.conf` 로 설정해 둔다.
5. (선택) Khadas 시간대: `sudo timedatectl set-timezone Asia/Seoul`

**혼동 주의**: 프롬프트가 `PS C:\...` 이면 노트북, `khadas@Khadas` 이면 Khadas 다. `scp` 는 노트북에서, `arduino-cli` · `teensy_loader_cli` · `picocom` 은 Khadas 에서 입력한다. `<Khadas IP>` 는 실제 주소로 바꿔 적는다.

## 1단계 — Khadas 도구 확인 (최초 1회)

| 항목 | 확인 명령 | 현재 상태 / 조치 |
|---|---|---|
| arduino-cli | `arduino-cli version` | `~/arduino/bin/arduino-cli` (1.5.1). PATH 추가: `echo 'export PATH="$HOME/arduino/bin:$PATH"' >> ~/.bashrc && source ~/.bashrc` |
| Teensy 코어 | `arduino-cli core list` | `teensy:avr 1.62.0` 설치됨 |
| teensy_loader_cli | `which teensy_loader_cli` | `/usr/bin/teensy_loader_cli` |
| 시리얼 권한 | `groups` | `dialout` 포함됨 |
| Teensy udev 규칙 | `ls /etc/udev/rules.d/ \| grep -i teensy` | `00-teensy.rules` 있음 |
| 시리얼 모니터 | `which picocom` | `sudo apt install -y picocom`. `arduino-cli monitor` 는 이 보드에서 동작하지 않음(No monitor available) |
| 플롯용 | `python3 -c "import matplotlib"` | 없으면 `sudo apt install -y python3-matplotlib` |

코드 폴더는 `~/arduino/` 아래에 스케치별로 있다: `dm_config_tool`, `dm_dual_test`, `hip_step_test`. 빌드 결과물은 `~/arduino/build/<스케치명>/`, 로그와 플롯 스크립트는 `~/logs/` 에 둔다.

## 2단계 — 코드를 Khadas 로 옮기기

스케치 파일 이름은 폴더 이름과 같아야 빌드된다 (`hip_step_test/hip_step_test.ino`).

**방법 A — scp**

1. Khadas: `mkdir -p ~/arduino/hip_step_test ~/logs`
2. 노트북 탭(`PS C:\...`)에서:
   ```powershell
   cd $HOME\Downloads
   scp .\hip_step_test.ino khadas@<Khadas IP>:~/arduino/hip_step_test/
   scp .\plot_log.py khadas@<Khadas IP>:~/logs/
   ```
3. `100%` 가 뜨면 성공. 덮어쓸 때도 같은 명령.

**방법 B — 저장소에서 바로** (Khadas)

```bash
git clone https://github.com/muneunhyeok/capstone-design_TWLR_KAU ~/twlr_repo   # 처음 한 번
cd ~/twlr_repo && git pull
cp -r firmware/bringup/hip_step_test ~/arduino/ && cp firmware/bringup/hip_step_test/plot_log.py ~/logs/
```

**방법 C — VS Code Remote-SSH (추천)**: 확장 "Remote - SSH" → 왼쪽 아래 `><` → Connect to Host → `khadas@<Khadas IP>` → Open Folder `/home/khadas/arduino/hip_step_test`. 파일을 고치고 저장하면 Khadas 에 반영되고, 아래 터미널(Ctrl + `)도 Khadas 터미널이다.

**동작하지 않았던 방법**: 긴 코드를 `cat > 파일 <<'INO_END'` 로 터미널에 붙여넣기. 첫 줄만 들어가고 `>` 상태에서 멈춘다 (`Ctrl+C` 로 빠져나옴).

(선택) 비밀번호 없이 접속: 노트북에서 `ssh-keygen -t ed25519` 후
`type $env:USERPROFILE\.ssh\id_ed25519.pub | ssh khadas@<Khadas IP> "mkdir -p ~/.ssh && cat >> ~/.ssh/authorized_keys"`

## 3단계 — CAN 배선과 모터 ID 확인

모터가 응답하지 않으면 먼저 배선과 ID 를 확인한다. TX 는 TX 끼리, RX 는 RX 끼리 연결하며 UART 처럼 교차하지 않는다.

| Teensy 4.1 | SN65HVD230 | 비고 |
|---|---|---|
| 3.3V | 3V3 | 5V · VIN 금지 |
| GND | GND | **모터 전원 GND(−)와도 연결** |
| 22 (CTX1) | CTX | 나중에 CAN2(ODrive)는 1번 |
| 23 (CRX1) | CRX | CAN2 는 0번 |

트랜시버 CANH/CANL → DM L → DM R 순서로 데이지체인한다. 전원을 끈 상태에서 CANH–CANL 저항이 약 60 Ω 이면 종단이 정상이다.

**ID 스캔 (`dm_config_tool`)**

```bash
cd ~/arduino/dm_config_tool
arduino-cli compile --fqbn teensy:avr:teensy41 --output-dir ~/arduino/build/dm_config_tool .
teensy_loader_cli --mcu=TEENSY41 -w -v ~/arduino/build/dm_config_tool/dm_config_tool.ino.hex   # 흰 버튼
picocom -b 115200 --echo /dev/ttyACM0      # 모터 전원 ON → s
```

2026-11-16 스캔 결과 (전원을 다시 넣어도 설정 유지):

| 모터 | CAN ID | Master ID | 모드 | PMAX / VMAX / TMAX | Timeout |
|---|---|---|---|---|---|
| L | 0x01 | 0x11 | MIT | 12.5 / 10 / 28 | 0 (off) |
| R | 0x02 | 0x12 | MIT | 12.5 / 10 / 28 | 0 (off) |

`Scan done: 0 motors` 가 나오면 버스에 응답이 전혀 없다는 뜻이다. 소프트웨어가 아니라 배선, GND 공통 연결, 트랜시버 전원을 확인한다.

## 4단계 — 빌드 · 업로드 · 시리얼 연결

Khadas 에서 한 줄씩 입력한다. 업로드는 모터 전원을 끈 상태에서 한다.

```bash
cd ~/arduino/hip_step_test
arduino-cli compile --fqbn teensy:avr:teensy41 --output-dir ~/arduino/build/hip_step_test .
teensy_loader_cli --mcu=TEENSY41 -w -v ~/arduino/build/hip_step_test/hip_step_test.ino.hex
picocom -b 115200 --echo --logfile ~/logs/run1.txt /dev/ttyACM0
```

- `Memory Usage on Teensy 4.1:` 이 나오면 빌드 성공. `main file missing from sketch` 는 폴더에 `.ino` 가 없는 것(2단계).
- `Waiting for Teensy device...` 가 나오면 Teensy 의 **흰 버튼을 한 번** 누른다 (리셋이 아니라 업로드 모드 버튼). `Programming... Booting` 이면 완료.
- `?` 를 눌러 도움말이 나오면 통신 정상. 키는 누르는 즉시 전송되고 Enter 는 필요 없다. 도움말 세 번째 줄에 `# f LOAD mode (kp 80) …` 가 보이면 v2.
- picocom 종료는 `Ctrl+A` → `Ctrl+X`. 다시 업로드하기 전에 반드시 종료한다(포트는 한 프로그램만 사용).
- 시험마다 `run2.txt`, `lift1.txt` 처럼 로그 이름을 바꾼다.

## 5단계 — hip_step_test 시험

`e` 는 힘이 0 인 상태로 켜기만 하고, 자세를 고정하는 것은 `h` 다. + 방향은 접힘이며 2026-11-16 에 양쪽에서 측정해 확인했다. 키 표는 [`firmware/bringup/README.md` §4](../../firmware/bringup/README.md#4-hip_step_test--다리-장착-후-고관절-계단-스윕--토크-실측).

**다리 무부하 시험 (방향 · 추종 확인)**

1. 몸통을 받쳐 **두 바퀴가 공중에** 뜨게 한다. 다리를 끝까지 펴도 바닥에 닿지 않아야 한다.
2. `0` → 바퀴를 손으로 받치고 `e`
3. 두 다리를 **가운데**로 들어 올린 채 `h`. 그냥 두면 바퀴 무게 때문에 펼침 끝까지 내려간다.
4. `s` → 양쪽 `rel` 이 ±1° 이내인지 확인. 5° 이상이면 아직 하중이 걸려 있는 것이다.
5. `g` 또는 `n` → 끝나면 `d`

**토르소 들기 시험 (v2, 바퀴 접지)**

1. 토르소 고정을 풀고, 한 사람이 앞뒤 기울어짐만 막는다. 토르소 밑에는 쪼그린 높이의 받침을 둔다.
2. `0` → `e` → (받침에 얹힌 자세 그대로) `h` → `f`
3. `]` 를 한 번씩 누르며 `s` 로 `ff=` 를 확인한다. **토르소가 뜨기 시작하는 ff 값을 기록**한다.
4. `n` 으로 들어 올리고 내린다 → `[` 를 여러 번 눌러 0 으로 → `f` → `d`

**자동 정지 조건 (양쪽 모두 OFF)**: 피드백 50 ms 끊김, 모터 에러 코드, 토크 14 N·m 초과(v1 은 10), 기준에서 22° 초과, 추종 오차 10° 초과, 온도 70 ℃ 초과.

## 6단계 — 로그 플롯

스윕 중 100 Hz 로 `D,t_ms,L_ref,L_pos,L_vel,L_tau,R_ref,R_pos,R_vel,R_tau,ff` 줄이 찍힌다. 각도 단위는 기준 대비 °, 토크는 N·m 이고 마이너스는 펼침 방향이다.

1. picocom 종료 (`Ctrl+A` → `Ctrl+X`)
2. `python3 ~/logs/plot_log.py ~/logs/run1.txt` → `run1.png`(각도 · 토크 · 추종오차 3단), `run1.csv`, 단계별 평균 토크 표
3. 노트북으로 가져오기: `scp khadas@<Khadas IP>:~/logs/run1.png .`

플롯 스크립트 없이 0.5 초 간격으로 요약:

```bash
grep '^D' ~/logs/run1.txt | awk -F, 'NR%50==1 {printf "t=%5.1fs  ref=%6.2f  L=%6.2f (%+.2f Nm)  R=%6.2f (%+.2f Nm)\n",$2/1000,$3,$4,$6,$8,$10}'
```

## 겪은 문제와 해결

| 증상 | 원인 | 해결 |
|---|---|---|
| tmux 를 켜면 `61;4;6;7;…c` 글자가 입력줄에 생김 | 터미널 응답이 SSH 로 늦게 도착해 입력으로 인식됨 | `Ctrl+U`, `set -sg escape-time 50` |
| `arduino-cli: command not found` | `~/arduino/bin` 이 PATH 에 없음 | `.bashrc` 에 PATH 추가 |
| `No monitor available for the port protocol serial` | 기본 시리얼 모니터 도구가 없음 | picocom 사용 |
| `-bash: Khadas: No such file or directory` | 노트북용 `scp` 를 Khadas 에서, `<Khadas IP>` 글자 그대로 입력 | 노트북 탭(`PS C:`)에서 실제 IP 로 입력 |
| PowerShell 에서 `arduino-cli` 인식 불가 | Khadas 용 명령을 노트북에 입력 | `ssh` 접속 후 입력 |
| `main file missing from sketch` | 폴더만 있고 `.ino` 파일이 없음 | scp 로 파일 전송 |
| `no feedback (frames received 0)`, 값이 전부 0, `T=0` | CAN 프레임을 하나도 못 받음 (배선) | 배선 점검 후 `dm_config_tool` 스캔으로 확인 |
| enable 후 LED 가 초록으로 안 바뀜 | enable 명령이 모터까지 못 감 | 위와 같음 |
| `dm_dual_test` 에서 `+` 를 눌러도 거의 안 움직임 | 유지 게인 Kp 5 → 0.1 rad 에 0.5 N·m 뿐 | `hip_step_test` (Kp 30) |
| `g` 복귀 중 약 10° 에서 멈춤 → `tracking error` | 바퀴가 바닥에 닿아 몸통 무게를 들어야 함 | 바퀴를 띄우거나 들기 모드 사용 |
| `n` 을 누르니 오히려 접히고 멈춤 | `h` 뒤 몸무게로 +5 ~ 7° 처짐(토크 −2.8 ~ −3.4 N·m), 펴는 힘 부족 | 위와 같음 |
| `n` 에서 −0.9° 에 막힘 (토크 약 −4.8 N·m) | 바퀴가 땅에 닿은 채 고정된 토르소 → 펼 공간 없음 | 바퀴를 띄우거나 토르소 고정 해제 + 들기 모드 |
| `dm_config_tool` 에서 `s` 를 눌렀는데 `SPIN stopping` 만 나옴 | 아직 `dm_dual_test` 가 올라가 있었음 (그 스케치의 `s` = 스핀 정지) | 원하는 스케치를 먼저 업로드 |

## 측정 결과와 다음 할 일

두 모터의 CAN 동시 구동과 방향 부호를 확인했고, 토르소 고정을 풀고 들기 모드로 바퀴 접지 상태 접힘 · 펼침에 성공했다.

- **방향**: 다리 방향 + = 접힘. L dir +1, R dir −1 가정이 맞음 (같은 명령에 양쪽이 같은 방향으로 접힘).
- **ID 저장**: 전원을 다시 넣어도 L 1/0x11, R 2/0x12 유지. 레지스터 저장 동작 확인됨.
- **막혔을 때 토크**: 추종 오차 약 9° 에서 −4.5 ~ −5.1 N·m (= Kp 30 × 오차). 모터는 명령대로 힘을 냈고, 무게나 바닥에 막힌 것이다.
- **`dm_dual_test` 수정**: `BENCH_SPIN_ALLOWED = false` (다리 장착 후 연속 회전 금지).

다음 할 일:

- [ ] 바퀴를 띄운 무부하 상태에서 `g` · `n` 양방향 스윕 + 플롯
- [ ] v2 들기 모드로 **토르소가 뜨기 시작하는 ff(N·m) 실측** → 통합 펌웨어 중력보상값의 근거
- [ ] 고관절 기계식 스토퍼 설계 (DM4340P 내장 없음, 펼침 −22° / 접힘 +22 ~ 25°)
- [ ] DM 타임아웃 레지스터 100 ms 설정 (현재 0 = 꺼짐)
- [ ] ODrive 휠을 CAN2(핀 1/0)로 옮기기: node 0 = L, node 1 = R, 1 Mbps
