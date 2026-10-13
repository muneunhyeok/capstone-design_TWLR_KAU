# twlr_control

TWLR 1단계 밸런싱 제어기 (다리 고정, 캐스케이드 PID). Gazebo와 실물에서 같은 노드를 씁니다.

## 구조
```
/imu/data ──────┐                       ┌─> /wheel_L/cmd_torque
/joint_states ──┼─> balance_controller ─┤
/cmd_vel ───────┘                       └─> /wheel_R/cmd_torque
                                         └─> /balance/debug
```
1. 속도 루프(PI): 목표 속도와 실제 속도 차이 → 목표 pitch (균형 각도 5.8° + 기울임)
2. pitch 루프(PD): 목표 pitch와 실제 pitch 차이 → 바퀴 토크
3. yaw 루프(PI): 회전 속도 차이 → 좌우 토크 차이
4. 넘어짐 감지: 균형 각도에서 35° 넘게 기울면 모터 끔. 5° 안으로 다시 세우면 자동 재시작

## 빌드
```bash
cd ~/twlr
colcon build --packages-select twlr_description twlr_control --symlink-install
source install/setup.bash
```

## 실행
```bash
ros2 launch twlr_control balance_sim.launch.py          # Gazebo + 제어기
```
키보드로 움직이기 (새 터미널):
```bash
sudo apt install ros-jazzy-teleop-twist-keyboard          # 처음 한 번
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
`i` 전진, `,` 후진, `j`/`l` 회전, `k` 정지. 최대 속도는 제어기에서 0.4 m/s로 제한.

## 게인 바꾸기 (실행 중)
```bash
ros2 param set /balance_controller kp_pitch 40.0          # 소수점 꼭 붙이기 (40 X, 40.0 O)
ros2 param get /balance_controller kp_pitch
ros2 param set /balance_controller enabled false          # 모터 끄기
```
마음에 드는 값은 `config/balance_sim.yaml`에 적어두면 다음 실행부터 적용.

## 그래프로 보기
PlotJuggler에서 아래 토픽을 구독하면 이름이 붙은 채로 보입니다.

| 토픽 | 내용 |
|---|---|
| /balance/pitch_deg | pitch [deg] (+ = 앞으로 숙임) |
| /balance/pitch_ref_deg | 목표 pitch [deg] |
| /balance/speed | 속도 [m/s] |
| /balance/speed_ref | 목표 속도 [m/s] |
| /balance/torque_L, /balance/torque_R | 왼쪽, 오른쪽 토크 [N·m] |
| /balance/yaw_rate | 회전 속도 [rad/s] |
| /balance/fallen | 넘어짐 1 / 정상 0 |

같은 값이 배열로도 나옵니다 (/balance/debug):
| 인덱스 | 내용 |
|---|---|
| data[0] | pitch [deg] (+ = 앞으로 숙임) |
| data[1] | 목표 pitch [deg] |
| data[2] | 속도 [m/s] |
| data[3] | 목표 속도 [m/s] |
| data[4], [5] | 왼쪽, 오른쪽 토크 [N·m] |
| data[6] | yaw 회전 속도 [rad/s] |
| data[7] | 넘어짐 1 / 정상 0 |

## 튜닝 순서 (증상 → 조정)
| 증상 | 조정 |
|---|---|
| 서지 못하고 바로 넘어짐 | kp_pitch 올리기 |
| 빠르게 떨림 (부들부들) | kd_pitch 내리기, 또는 kp_pitch 내리기 |
| 천천히 크게 앞뒤로 출렁임 | kd_pitch 올리기 |
| 계속 한쪽으로 굴러감 | ki_vel 올리기 |
| 제자리로 돌아올 때 크게 왔다갔다 | kp_vel 올리기, ki_vel 내리기 |

## 실물로 옮길 때 바꿀 것
- `use_sim_time` 끄기, `pitch_offset_deg`를 실측 균형 각도로
- 바퀴 방향이 반대인 쪽은 `left_wheel_sign` / `right_wheel_sign`을 -1.0으로
- `/imu/data`, `/joint_states`, `/wheel_*/cmd_torque`를 실제 하드웨어 노드가 같은 이름·단위로 주고받으면 제어기는 수정 없이 동작
