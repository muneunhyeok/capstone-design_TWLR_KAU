# 01 — Gazebo 시뮬레이션 구성 · 실행 · 값

대상 환경: Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic (gz-sim 8). 코드는 [`sim/`](../../sim/) 에 있습니다.

## 1. 패키지

| 패키지 | 역할 | 주요 파일 |
|---|---|---|
| [`sim/twlr_description`](../../sim/twlr_description/) | 모델 · 월드 · 브리지 | `urdf/twlr.urdf.xacro` · `worlds/twlr_world.sdf` · `config/gz_bridge.yaml` · `launch/sim.launch.py` · `launch/display.launch.py` · `meshes/*.stl`(10종, 5 MB) · `scripts/wheel_torque_test.py` |
| [`sim/twlr_control`](../../sim/twlr_control/) | 1단계 균형 제어기 | `twlr_control/balance_core.py`(ROS 무관 수식) · `balance_node.py` · `config/balance_sim.yaml` · `launch/balance_sim.launch.py` |

URDF 는 CATIA 어셈블리 `assembly_urdfchange_.stp` 에서 자동 생성했습니다. 저장소 루트의 `urdf/`(구버전 Fusion 자동생성, 63.5 kg)와는 **별개**입니다.

## 2. 설치 · 실행

저장소 루트가 이미 ROS 패키지(`TWLR_description`)라서 colcon 이 그 안쪽의 `sim/` 패키지를 인식하지 못할 수 있습니다. **두 폴더를 워크스페이스로 복사**해서 쓰세요.

```bash
sudo apt install ros-jazzy-xacro ros-jazzy-robot-state-publisher ros-jazzy-joint-state-publisher-gui \
                 ros-jazzy-rviz2 ros-jazzy-ros-gz ros-jazzy-teleop-twist-keyboard
mkdir -p ~/twlr/src && cp -r sim/twlr_description sim/twlr_control ~/twlr/src/
cd ~/twlr && colcon build --symlink-install && source install/setup.bash

ros2 launch twlr_control balance_sim.launch.py        # Gazebo + 제어기
ros2 run teleop_twist_keyboard teleop_twist_keyboard  # i 전진 , 후진 j/l 회전 k 정지
```

| 목적 | 명령 |
|---|---|
| 모델 확인 (RViz, 물리 없음) | `ros2 launch twlr_description display.launch.py` (`legs_fixed:=false` 로 다리 슬라이더) |
| Gazebo 만 (제어기 없음, 넘어지는 게 정상) | `ros2 launch twlr_description sim.launch.py` |
| 아크릴 없이 | `... acrylic_mass:=0.001` — 이때 `pitch_offset_deg` 도 8.2 로 (README 명령만으론 5.8 이 남음) |
| 바퀴 토크 부호 확인 | `ros2 run twlr_description wheel_torque_test.py --ros-args -p torque:=0.5 -p duration:=2.0` |
| 게인 실시간 변경 | `ros2 param set /balance_controller kp_pitch 40.0` (소수점 필수) · 영구 변경은 `balance_sim.yaml` |

런치 인자

| 인자 | 기본값 | 설명 |
|---|---|---|
| `acrylic_mass` | 0.515 kg | 아크릴 커버 질량 (CAD 433 cm³ × 1.19 g/cm³). 0.001 = 커버 없음 |
| `legs_fixed` | true | true: 힙·무릎·커플러 용접(1단계). false: 다리 이동(2단계, 4절 closure 포함). `sim.launch.py` 에서만 지정 |
| `spawn_pitch` | `sim.launch.py` 0.0 / `balance_sim.launch.py` **0.101 rad** | 초기 pitch (+ = 앞으로 숙임). 0.101 = 균형각 5.8° |

## 3. ROS 토픽

| 토픽 | 방향 | 형식 | 내용 |
|---|---|---|---|
| `/clock` | Gazebo → ROS | `rosgraph_msgs/Clock` | 시뮬 시간 (모든 노드 `use_sim_time`) |
| `/imu/data` | Gazebo → ROS | `sensor_msgs/Imu` | 400 Hz, `imu_link`, 방향은 ENU 기준(BNO085 게임 로테이션 벡터에 대응) |
| `/joint_states` | Gazebo → ROS | `sensor_msgs/JointState` | 바퀴(다리 이동 모드에선 다리 포함) 각도 · 각속도 |
| `/wheel_L/cmd_torque` · `/wheel_R/cmd_torque` | ROS → Gazebo | `std_msgs/Float64` | 바퀴 토크 [N·m], **+ = 전진** (좌우 동일) |
| `/cmd_vel` | → 제어기 | `geometry_msgs/Twist` | `linear.x` 속도, `angular.z` 선회 (0.5 s 타임아웃) |
| `/balance/{pitch_deg, pitch_ref_deg, speed, speed_ref, torque_L, torque_R, yaw_rate, fallen}` | 제어기 → | `Float64` | PlotJuggler 용 (같은 값이 `/balance/debug` 배열로도 나옴) |

Gazebo 쪽 토픽은 `/model/twlr/joint/wheel_{L,R}_joint/cmd_force`(`ApplyJointForce`) 와 `/imu` · `/joint_states` 입니다.

## 4. 좌표계 · 치수

| 항목 | 값 |
|---|---|
| 좌표 | ROS REP-103: x = 전진, y = 왼쪽, z = 위. CATIA → ROS: x = −y_c, y = +x_c, z = +z_c |
| `base_link` 원점 | 좌우 **힙모터 축의 중점**. 모든 관절각 0 = CATIA 어셈블리 자세 |
| 바퀴 | 반지름 **86 mm**, 폭 52 mm, 좌우 간격 **488.2 mm**(y = ±244.1), 바퀴축 base 기준 (x −4, z −336) mm |
| 링크 길이 (측면 평면) | thigh 239.1 · shin 241.8 · coupler 211.3 · 몸통(hip→coupler 피벗) 124.1 mm |
| 충돌 형상 | 몸통 박스 0.317 × 0.314 × 0.144 m @ (0.0215, 0, 0.013), 바퀴 원통. 다리는 충돌 없음 |
| IMU | `imu_link` = (0.0001, −0.0001, 0.0561), `base_link` 와 같은 방향 가정 |
| 월드 | 물리 1 ms, g = 9.81, 지면 50 × 50 m, 마찰 μ = 1.0 |

## 5. 질량 · 무게중심 (URDF)

아크릴 포함 총 **9.62 kg**, 제외 **9.10 kg**. 출력물 = 슬라이서(PLA 인필 20 %), 구매품 = 판매처, 나사 = 개수 × 개당. **저울 실측은 없음.**

| 링크 | 질량 [kg] | 무게중심 (링크 원점 기준 x, y, z) [m] |
|---|---|---|
| `base_link` | 2.807 | +0.0113, −0.0022, +0.0015 |
| `acrylic_link` | 0.515 (인자) | +0.0646, 0.0000, +0.0499 |
| `thigh_L` / `thigh_R` | 0.338 | −0.0961, ±0.0038, −0.0906 |
| `coupler_L` / `coupler_R` | 0.137 | −0.0564, ±0.0129, −0.0875 |
| `shin_L` / `shin_R` | 0.473 | +0.0393, +0.0150, −0.0432 / +0.0386, −0.0152, −0.0424 |
| `wheel_L` / `wheel_R` | 2.200 | 0 (WA172E 허브모터 전체를 회전부로 포함 → 바퀴 관성 과대) |

| 파생값 (검증 스크립트로 재계산) | 아크릴 포함 | 아크릴 제외 |
|---|---|---|
| 몸통+다리 무게중심, 바퀴축 기준 | 29.4 mm **뒤**, 290.5 mm 위 | 40.1 mm 뒤, 280.1 mm 위 |
| **균형 pitch** (+ = 앞으로 숙임) | **5.79°** | **8.16°** |
| 전체 무게중심 지면 높이 | 243.6 mm | 230.7 mm |
| 몸통+다리 / 바퀴 질량 | 5.218 / 4.400 kg | 4.704 / 4.400 kg |

> IMU pitch 0° 는 균형점이 아닙니다. 이 다리 자세에서는 몸통이 앞으로 약 5.8° 숙인 채 섭니다.

**임시값** (확정 후 xacro 상단 · `base_link` 에서 수정)

| 항목 | 임시값 | 근거 |
|---|---|---|
| 위치 미정 부품 168 g (강압기 70 · USB 30 · 전선 40 · XT90 20 · CAN 3 · 분배기 5) | 몸통 앞 (x +0.05, z 0) 점질량 | 위치 미정 |
| 힙 각도 범위 | ±0.8 rad, effort 27 N·m, 10 rad/s | DM4340P 피크 27 N·m (데이터시트 확인 필요) |
| 휠 토크 한계 | **10 N·m** = ODrive 10 A × Kt 1 | WA172E 정격 5 / 정지 13 N·m, 무부하 210 rpm(22 rad/s) |
| 전장품 외형 | 박스 · 원통으로 단순화 (질량 · 무게중심은 실제 값) | |
| 힙모터 | 전체를 몸통에 포함 | |

## 6. 제어기 (`balance_core.py`)

캐스케이드 PID. 같은 노드를 Gazebo 와 실물에서 쓰도록 `/imu/data` · `/joint_states` · `/wheel_*/cmd_torque` 이름·단위만 맞추면 됩니다.

```
속도 PI (v_ref − v) ──▶ 목표 pitch = offset + lean ──▶ pitch PD ──▶ 바퀴 토크 합
yaw PI (yaw_cmd − yaw_rate) ──▶ 좌우 차동          넘어짐: |pitch−offset| > 35° → 모터 끔, 5° 이내로 세우면 재시작
```

| 파라미터 | 기본값 | 단위 · 비고 |
|---|---|---|
| `pitch_offset_deg` | 5.8 | 균형각 (아크릴 제외 8.2) |
| `kp_pitch` · `kd_pitch` | 30 · 4 | N·m/rad · N·m·s/rad, 두 바퀴 합 |
| `kp_vel` · `ki_vel` · `vel_int_limit` | 0.3 · 0.5 · 2.0 | rad/(m/s) · rad/m · m |
| `max_lean_deg` | 15 | |
| `kp_yaw` · `ki_yaw` · `yaw_int_limit` | 0.5 · 1.0 · 0.5 | |
| `max_wheel_torque` | 10 | 휠당 N·m |
| `wheel_radius` · `vel_filter_tau` | 0.086 · 0.02 | m · s |
| `max_speed` · `max_accel` · `max_yaw_rate` | 0.4 · 0.5 · 1.5 | m/s · m/s² · rad/s |
| `fall_angle_deg` · `rearm_angle_deg` | 35 · 5 | |
| `left_wheel_sign` · `right_wheel_sign` · `cmd_timeout` | 1 · 1 · 0.5 | 실물 모터 방향 반전용 |

부호 규약: pitch > 0 = 앞으로 숙임 · 바퀴 속도 > 0 = 전진 · 바퀴 토크 > 0 = 전진 · yaw rate > 0 = 좌회전. 속도 추정은 `v = r (평균 바퀴 각속도 + pitch_rate)` (바퀴 엔코더는 다리 기준이라 몸통 pitch rate 를 더해 지면 기준으로 환산).

튜닝 증상표와 PlotJuggler 토픽은 [`sim/twlr_control/README.md`](../../sim/twlr_control/README.md) 에 있습니다.

## 7. 2단계 (다리 이동)

`legs_fixed:=false` 에서는 힙 · 무릎 · 커플러가 회전 관절이 되고 coupler–shin 사이에 4절 폐루프 closure 조인트(raw SDF)가 추가됩니다. closure 위치는 0 자세에서 정확히 닫히는 것을 확인했지만([02 §2](02-verification-and-open-items.md#2-확인된-것)), **힙을 구동하는 플러그인이 없고 Gazebo 의 폐루프 처리는 미검증**입니다 → [02 §4](02-verification-and-open-items.md#4-열린-항목).
