# 호스트 측 코드 (LattePanda / Ubuntu)

| 파일 | 내용 | 출처 |
|---|---|---|
| [`imu_serial_reader.py`](imu_serial_reader.py) | Teensy 4.1 이 보내는 BNO085 쿼터니언 CSV(`i,j,k,real`) 수신·파싱 | 3주차 발표 슬라이드 22 |
| [`odrive_wheel_config.odrivetool.py`](odrive_wheel_config.odrivetool.py) | 휠모터 ODrive 설정 (pole_pairs 15, 인크리멘털 엔코더 cpr 3200, 시작 게인) — `odrivetool` 셸에 붙여넣기 | 3주차 발표 슬라이드 19 |

Teensy 펌웨어: [`../firmware/teensy_bno085_quat/`](../firmware/teensy_bno085_quat/)

```
BNO085 ──I²C──▶ Teensy 4.1 ──USB 시리얼 115200──▶ LattePanda (imu_serial_reader.py)
                    │
                    └──CAN 1 Mbps (트랜시버)──▶ DM4340P × 2   (계획)
LattePanda ──USB──▶ ODrive ──▶ WA172E 휠 × 2
```

다음 단계: 쿼터니언 → Roll/Pitch/Yaw 변환 후 ROS 2 토픽(`sensor_msgs/Imu`)으로 발행, 밸런싱 제어 입력으로 연결.
