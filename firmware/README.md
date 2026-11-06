# Teensy 4.1 펌웨어

| 폴더 | 내용 |
|---|---|
| [`bringup/dm_single_test/`](bringup/) | DM-J4340P-2EC 1개를 Teensy CAN1 로 구동하는 단품 시험(활성화 · 정속 회전 · 정지). 현황 · 미해결 항목은 [`bringup/README.md`](bringup/README.md) |
| [`teensy_bno085_quat/`](teensy_bno085_quat/) | BNO085 IMU(I²C) 의 Game Rotation Vector 를 읽어 쿼터니언 `i,j,k,real` 을 USB 시리얼 115200 으로 CSV 출력. Arduino IDE + Teensyduino, 라이브러리 `Adafruit BNO08x` |

호스트 수신 코드: [`../host/imu_serial_reader.py`](../host/imu_serial_reader.py)

## 다음 — 통합 펌웨어 (계획)

DM4340P(CAN1) · ODrive(CAN2) · BNO085(I²C)를 Teensy 하나에서 500 Hz 로 묶는 통합 펌웨어(`twlr_teensy/`)와
단품 시험 스케치 중 `bringup/dm_single_test` 는 추가했고 `bringup/odrive_can_test` 는 추가할 예정입니다.
구조 · 상태머신 · 제어 법칙 · `config.h` 값은 [`docs/electronics/01`](../docs/electronics/01-can-integration-plan.md#7-teensy-펌웨어-구조)에,
코드 해설은 [`docs/electronics/02`](../docs/electronics/02-control-principles.md)에 정리했습니다.
