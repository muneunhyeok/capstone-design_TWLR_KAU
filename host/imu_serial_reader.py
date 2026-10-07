"""BNO085 쿼터니언 수신·파싱 (Ubuntu 호스트 / LattePanda) — Teensy 4.1 USB 시리얼 CSV "i,j,k,real".
   3주차 발표 슬라이드 22 코드 그대로. 사용: pip install pyserial && python3 imu_serial_reader.py
"""
import serial
import sys
import time

# 우분투 환경에 맞는 시리얼 포트 설정
SERIAL_PORT = '/dev/ttyACM0'  # 필요시 ttyACM1로 변경
BAUD_RATE = 115200

def main():
    print(f"Connecting to {SERIAL_PORT} at {BAUD_RATE} baud...")
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2) # 연결 안정화 대기
    except Exception as e:
        print(f"Failed to connect to serial port: {e}")
        sys.exit(1)

    print("Listening for BNO085 IMU data... Press Ctrl+C to exit.")
    
    try:
        while True:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    # 쉼표(,)로 데이터 분리
                    parts = line.split(',')
                    if len(parts) == 4:
                        try:
                            qx = float(parts[0])
                            qy = float(parts[1])
                            qz = float(parts[2])
                            qw = float(parts[3])
                            
                            # 수신된 쿼터니언 데이터 출력 (제어 알고리즘 및 ROS 2 연동 지점)
                            print(f"Quaternion -> X: {qx:.3f}, Y: {qy:.3f}, Z: {qz:.3f}, W: {qw:.3f}")
                        except ValueError:
                            pass
    except KeyboardInterrupt:
        print("\nExiting program...")
    finally:
        ser.close()

if __name__ == '__main__':
    main()
