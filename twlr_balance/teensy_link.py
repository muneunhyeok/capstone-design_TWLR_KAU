"""P2: Teensy 시리얼 프로세스 - IMU/힙 상태 받기, 힙 명령 보내기.
프로토콜 (firmware/twlr_bridge.ino 와 짝)
  Teensy -> PC : A5 5A | <I 24f B B B B> (104 B) | sum8     = 107 B, 250 Hz   (펌웨어 v3)
                 [0] t_us, [1..6] roll, pitch, yaw, gx, gy, gz [rad, rad/s]  (gy = pitch 각속도, pitch 와 같은 부호)
                 [7..12] hipL, hipR [deg, h 기준, dir 적용], hipVL, hipVR [deg/s], hipTL, hipTR [N*m],
                 [13..15] loopHz, imuUsMax [us], imuEvHz  (Teensy 진단)
                 [16..22] th_com, w_com, v_meas, v_cmd, x, th_ref, trim_deg  (Teensy 밸런싱 제어기 상태)
                 [23..24] ODrive 하트비트 Hz, 엔코더 응답 Hz
                 [25] flags, [26] imu_ok, [27] bal_mode, [28] bal_err
  PC -> Teensy : 5A A5 | <c f f> (9 B) | sum8           = 12 B
                 'E' enable(FREE)  'H' hold here  'D' disable all  'K' kp kd  'T' tffL tffR  'W' heartbeat
"""
import struct, time, queue
import shared
from shared import I

RX_HDR = b"\xA5\x5A"
RX_FMT = "<I24fBBBB"
RX_LEN = struct.calcsize(RX_FMT)          # 104
# 패킷 튜플 인덱스
IX_THCOM, IX_WCOM, IX_VMEAS, IX_VCMD, IX_X, IX_THREF, IX_TRIM, IX_HBHZ, IX_ENCHZ = range(16, 25)
IX_FLAGS, IX_IMUOK, IX_MODE, IX_ERR = 25, 26, 27, 28
TX_HDR = b"\x5A\xA5"

def encode_cmd(c, a=0.0, b=0.0):
    p = struct.pack("<cff", c.encode() if isinstance(c, str) else c, float(a), float(b))
    return TX_HDR + p + bytes([sum(p) & 0xFF])

class Parser:
    def __init__(self):
        self.buf = bytearray()
    def feed(self, data):
        self.buf += data
        out = []
        while True:
            i = self.buf.find(RX_HDR)
            if i < 0:
                del self.buf[:-1]; break
            if len(self.buf) - i < 2 + RX_LEN + 1:
                del self.buf[:i]; break
            p = bytes(self.buf[i+2:i+2+RX_LEN]); chk = self.buf[i+2+RX_LEN]
            if (sum(p) & 0xFF) == chk:
                out.append(struct.unpack(RX_FMT, p)); del self.buf[:i+3+RX_LEN]
            else:
                del self.buf[:i+1]                 # 헤더 오인식 → 한 바이트 넘기고 다시
        return out

def run(buf, hip_cmd_q, port, baud):
    import serial
    ser = serial.Serial(port, baud, timeout=0.002)
    par = Parser(); t_hb = 0.0
    try:
        while not buf[I.QUIT]:
            data = ser.read(512)
            if data:
                pkts = par.feed(data)
                if pkts:
                    pk = pkts[-1]                                                         # 최신만
                    (t_us, r, p, y, gx, gy, gz, hl, hr, vl, vr, tl, tr, lhz, ius, iev) = pk[:16]
                    fl, ok = pk[IX_FLAGS] & 0x0F, pk[IX_IMUOK]
                    shared.put(buf, T_TEENSY=time.monotonic(), ROLL=r, PITCH=p, YAW=y, GX=gx, GY=gy, GZ=gz,
                               HIP_L=hl, HIP_R=hr, HIP_VL=vl, HIP_VR=vr, HIP_TL=tl, HIP_TR=tr,
                               T_LOOP_HZ=lhz, T_IMU_US=ius, T_IMU_HZ=iev, HIP_FLAGS=fl, IMU_OK=ok)
            while True:                                  # 명령 비우기 (안 기다림)
                try:
                    c, a, b = hip_cmd_q.get_nowait()
                except queue.Empty:
                    break
                ser.write(encode_cmd(c, a, b))
            now = time.monotonic()
            if now - t_hb > 0.1:
                ser.write(encode_cmd('W')); t_hb = now
    finally:
        try:
            ser.write(encode_cmd('D'))                   # 종료 시 힙 끄기
        except Exception:
            pass
        ser.close()
