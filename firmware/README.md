# Teensy 4.1 펌웨어

| 폴더 | 내용 |
|---|---|
| [`teensy_bno085_quat/`](teensy_bno085_quat/) | BNO085 IMU(I²C) 의 Game Rotation Vector 를 읽어 쿼터니언 `i,j,k,real` 을 USB 시리얼 115200 으로 CSV 출력. Arduino IDE + Teensyduino, 라이브러리 `Adafruit BNO08x` |

호스트 수신 코드: [`../host/imu_serial_reader.py`](../host/imu_serial_reader.py)
