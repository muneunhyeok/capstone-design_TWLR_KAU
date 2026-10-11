# 호스트 측 코드 (LattePanda / Ubuntu)

| 파일 | 내용 | 출처 |
|---|---|---|
| [`imu_serial_reader.py`](imu_serial_reader.py) | Teensy 4.1 이 보내는 BNO085 쿼터니언 CSV(`i,j,k,real`) 수신·파싱 | 3주차 발표 슬라이드 22 |
| [`odrive_wheel_config.odrivetool.py`](odrive_wheel_config.odrivetool.py) | 휠모터 ODrive 설정 (axis0, USB 속도 제어 시험용) — `odrivetool` 셸에 붙여넣기 | 3주차 발표 슬라이드 19 |
| [`odrive_can_setup.odrivetool.py`](odrive_can_setup.odrivetool.py) | **위 설정을 2축 + CAN 토크 모드로 확장** (1 Mbps · node 0/1 · 엔코더 2 ms 송신 · 워치독 · 브레이크 저항) + 1회 모터 캘리브 | 통합본 슬라이드 63 · 88 · 101 |
| [`dm4340p_setup.md`](dm4340p_setup.md) | **DM4340P 1회 설정** — ID · Master ID · MIT 모드 · P/V/T MAX · CAN Timeout · 영점 | 통합본 슬라이드 62 · 101 |

Teensy 펌웨어: [`../firmware/`](../firmware/)

```
현재 (3주차)
BNO085 ──I²C──▶ Teensy 4.1 ──USB 시리얼 115200──▶ LattePanda (imu_serial_reader.py)
LattePanda ──USB──▶ ODrive ──▶ WA172E 휠 × 2

CAN 통합 후 (계획)
LattePanda (ROS 2 twlr_bridge) ──USB 패킷 Command 50 Hz / Telemetry 100 Hz──▶ Teensy 4.1 (500 Hz)
Teensy 4.1 ──CAN1 1 Mbps──▶ DM4340P × 2      ──CAN2 1 Mbps──▶ ODrive ──▶ WA172E × 2
BNO085 ──I²C 400 kHz──▶ Teensy 4.1
```

통합 구조 · 패킷 형식 · ROS 2 토픽: [`docs/electronics/01`](../docs/electronics/01-can-integration-plan.md) · 운용 순서: [`docs/electronics/03`](../docs/electronics/03-operation-manual.md)
