# 02 — 검증 결과 · CAD 와의 차이 · 열린 항목

검증일 2026-10-13. 재현: `python3 sim/verify/verify_sim.py` (필요: `pip install xacro numpy`, 약 1분) · 전체 출력: [`sim/verify/results/sim-verification.txt`](../../sim/verify/results/sim-verification.txt)

## 1. 무엇을 어떻게 확인했나

Gazebo 를 띄우지 않고 **URDF 숫자 · STL · 제어기 코드**만으로 확인했습니다. 제어기(`balance_core.py` 원본)를 URDF 의 질량 · 무게중심 · 관성으로 만든 **평면 바퀴-역진자 모델**(미끄럼 없음, 마찰 없음, 노이즈 없음, 제어 400 Hz)에 붙여 시뮬레이션했습니다.

> **한계** — 접촉 모델(바퀴-지면), 브리지 지연 지터, 센서 노이즈, 백래시는 들어 있지 않습니다. 아래 "여유"는 이상적인 경우이며 실제 Gazebo · 실물에서는 더 작을 수 있습니다.

## 2. 확인된 것

| 항목 | 결과 |
|---|---|
| xacro 변환 | 다리 고정 · 다리 이동 두 모드 모두 성공, 모든 링크 관성 텐서가 양정치 + 삼각부등식 만족 |
| 질량 · 무게중심 | 9.618 kg (아크릴 제외 9.104) · 균형 pitch **5.79°**(8.16°) · 전체 무게중심 지면 높이 243.6 mm — 「질량·무게중심 정리」 문서의 값과 일치 |
| 메시 | 단위 m. 바퀴 STL 은 지름 0.172 m 에 원점이 중심 → 스케일 · 원점 문제 없음 |
| 스폰 | 휠 바닥이 base 아래 0.422 m, 스폰 z 0.430 → 8 mm 낙하 (`spawn_pitch` 0.101 에서는 10 mm) |
| 4절 링크 | 측면 평면 길이 thigh 239.1 · coupler 211.3 · shin 241.8 · 몸통 124.1 mm (README 와 일치). closure 점이 0 자세에서 정확히 닫힘 (coupler 길이 211.3) |
| 부호 규약 | pitch · 토크 · 속도 추정이 서로 일관 (앞으로 숙임 → 전진 토크 → 몸통은 뒤로 복원) |

제어기 (기본 게인 kp 30 / kd 4, 평면 모델)

| 시나리오 | 결과 |
|---|---|
| 균형각 / 직립 0° / −6° / +15° 에서 시작 | 모두 5.79° 로 정착, 최대 토크 3.09 N·m/휠 (한계 10) |
| 0.25 m/s 밀침 · 0.6 m/s 밀침 | 정착 (최대 pitch 이탈 8.5° · 16.9°) |
| `v_cmd` 0.4 m/s (2~6 s) | 추종 후 정지 (1.60 m 이동) |
| 아크릴 제거 + offset 5.8 그대로 | 8.16° 로 수렴, 0.08 m 흘러감 |

| 여유 (0.25 m/s 밀침 후 정착 여부) | 안정 범위 |
|---|---|
| 센서 → 토크 지연 | **50 ms 까지** (60 ms 에서 발진) |
| `kp_pitch` (kd 4) | **18 ~ 45** (15 이하 · 60 이상 불안정) |
| `kd_pitch` (kp 30) | **3 ~ 8** (2 이하 · 12 이상 불안정) |
| 무게중심 높이 | ×0.7 ~ ×1.2 (×1.4 불안정, offset 5.8 유지 조건) |
| 몸통 질량 (관성 비례) | ×0.7 ~ ×2.0 |
| 휠당 토크 한계 | 0.8 N·m 이상 (0.5 이하 넘어짐) |

기본값(kp 30 · kd 4)은 kp 0.6 ~ 1.5 배, kd 0.75 ~ 2 배 범위 안에서만 안정입니다. 특히 kd 는 어느 쪽으로도 여유가 작아서 한 번에 조금씩만 바꾸는 것이 좋습니다.

## 3. 시뮬 URDF 와 CAD 모델의 차이

새 URDF(CATIA `assembly_urdfchange_.stp` 기반, 9.62 kg)와 이 저장소의 해석 기준 모델(Fusion `TWLR_assembly_AK45`, [`mass-and-cog.txt`](../../hardware/analysis/results/mass-and-cog.txt), 11.24 kg)은 **서로 다른 모델**이고 수치가 다릅니다. 어느 쪽을 기준으로 통일할지 정해지지 않아 두 값을 그대로 둡니다.

| 그룹 [kg] | 시뮬 URDF | CAD (`mass-and-cog.txt`) | 시뮬 − CAD |
|---|---|---|---|
| 몸통 (URDF: base + 아크릴 / CAD: BODY) | 3.322 | 4.615 | −1.293 |
| 허벅지 (CAD 는 허브 · 6905 베어링 포함) | 0.676 | 1.149 | −0.473 |
| 종아리 (CAD 는 6904 베어링 포함) | 0.946 | 0.900 | +0.046 |
| 커플러 | 0.274 | 0.176 | +0.098 |
| 바퀴 (WA172E × 2) | 4.400 | 4.400 | 0 |
| **합계** | **9.618** | **11.240** | **−1.622** |

| 그 밖 | 시뮬 URDF | CAD |
|---|---|---|
| 전체 무게중심 지면 높이 | 243.6 mm | 271.5 mm |
| 전체 무게중심 휠축 위 | 157.6 mm | 185.5 mm |
| 바퀴 좌우 간격 | 488.2 mm (±244.1) | 515.2 mm (±257.6) |
| 바퀴축 높이 (힙축 기준) | −336 mm | −336 mm (일치) |
| 허벅지 한 쪽 | 0.338 kg | 0.3425 kg (거의 일치) |

차이의 **원인은 확인하지 못했습니다.** 확인해야 할 후보:
- CAD 몸통 그룹에는 배터리 700 g · DM4340P 750 g 이 들어 있음 → 시뮬 `base_link` 2.807 kg 에 배터리가 포함됐는지
- CAD 허벅지 그룹의 허브(AL) 265 g · 6905 베어링 199 g 이 시뮬에 있는지 (6905 는 현재 무기능, [hardware §6-1](../../hardware/README.md#6-미해결--판단-필요))
- 시뮬 커플러(137 g)와 CAD 커플러(87 g)의 차이 — 부시 · 나사 포함 여부
- 바퀴 좌우 간격 27 mm 차이는 질량이 아니라 형상 차이

## 4. 열린 항목

1. **실제 Gazebo 구동 미확인** — 이 저장소의 검증은 계산과 평면 모델입니다. → §6 체크.
2. **조용한 실패** — `balance_node` 는 `/joint_states` 에 `wheel_L_joint`/`wheel_R_joint` 가 없거나 velocity 배열이 비면 `have_joints` 가 False 로 남아 **경고 없이 토크 0** 만 보냅니다. 일정 시간 뒤 경고를 내도록 고치는 것을 권합니다.
3. **아크릴 제거 시 offset** — `balance_sim.launch.py` 에 offset 인자가 없습니다. `acrylic_mass:=0.001` 과 함께 `pitch_offset_deg` 를 8.2 로 바꿔야 깔끔합니다.
4. **다리 이동 모드(2단계)** — 힙을 구동하는 플러그인이 없어(`ApplyJointForce` 는 바퀴만) 다리가 접힙니다. Gazebo(DART) 가 closure 폐루프를 제대로 풀어 주는지는 미검증입니다.
5. **바퀴 `kp` / `kd` 태그** — `<gazebo reference="wheel_*">` 의 `kp`, `kd` 는 Gazebo Classic 용이라 Harmonic 에서 효과가 없을 수 있습니다. 마찰은 `mu1`/`mu2` 와 월드 μ 로 정해집니다.
6. **질량 실측 없음** — 다음 단계는 로봇 전체 무게를 재서 9.62 kg(또는 CAD 11.24 kg)과 비교하는 것입니다.
7. **LQR 모델과의 차이** — [`docs/electronics/02`](../electronics/02-control-principles.md) 의 모델(추정 I_w 0.0085 · I_b 0.07)은 불안정 극점 ±7.59 rad/s, 이번 URDF 에서 선형화한 값은 **±6.54 rad/s**(아크릴 제외 ±6.55)입니다. 입력 이득도 다릅니다(각가속도 −8.33 vs −7.18 rad/s²/(N·m), 가속도 +2.34 vs +1.92 m/s²/(N·m)). K = [2.24, 4.10, 28.4, 4.59] 를 URDF(또는 CAD) 값으로 다시 계산할지 판단이 필요합니다. 단 §3 의 모델 차이를 먼저 정리해야 합니다.
8. **시뮬 제어기는 실물 계획과 다른 구조** — 시뮬은 ROS 안의 캐스케이드 PID(휠 토크 ±10 N·m), 실물 계획은 Teensy 500 Hz LQR(휠 토크 ±8 N·m, `/twlr/enable` · `/twlr/balance` 명령)입니다. 시뮬 게인은 실물로 그대로 옮길 수 없고, `sim/twlr_control/README.md` 의 "실물로 옮길 때"는 이 ROS 노드를 그대로 쓰는 경우에만 해당합니다.

## 5. 시뮬 제어기 vs 실물 계획 (요약)

| | 시뮬 (`sim/twlr_control`) | 실물 계획 ([전장 · 제어](../electronics/)) |
|---|---|---|
| 위치 | ROS 2 노드 (Python) | Teensy 4.1 펌웨어 500 Hz |
| 구조 | 속도 PI → pitch PD, yaw PI | LQR (x, ẋ, θ, θ̇) + yaw P |
| 휠 토크 한계 | 10 N·m / 휠 | 8 N·m / 휠 |
| 입력 | `/imu/data` · `/joint_states` · `/cmd_vel` | Teensy 내부 센서 · `/cmd_vel` |
| 출력 | `/wheel_{L,R}/cmd_torque` | CAN2 → ODrive 토크 모드 |

## 6. Gazebo 첫 실행 체크

```bash
ros2 launch twlr_control balance_sim.launch.py
ros2 topic hz /imu/data                       # ≈ 400 Hz
ros2 topic echo /joint_states --once          # name 에 wheel_L_joint · wheel_R_joint, velocity 배열이 채워져 있어야 함
ros2 topic echo /clock --once                 # 시뮬 시간
ros2 run twlr_description wheel_torque_test.py   # (제어기 끄고) + 토크가 전진 방향인지
```

세 번째가 비어 있으면 §4-2 에 해당합니다. 균형이 안 잡히면 `/balance/pitch_deg` 가 5.8° 근처에서 움직이는지, `/balance/fallen` 이 1 인지부터 보세요.
