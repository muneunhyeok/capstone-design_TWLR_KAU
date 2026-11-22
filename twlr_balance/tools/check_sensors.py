"""센서 확인 도구 - 바퀴는 절대 안 켬. 값이 잘 들어오는지 + 부호가 맞는지 화면으로 확인.
(Teensy 에 twlr_bridge 펌웨어가 올라가 있어야 함. odrivetool / picocom 은 끄고 실행)

  python3 tools/check_sensors.py              # Teensy + ODrive 둘 다
  python3 tools/check_sensors.py --no-odrive  # Teensy(IMU+힙)만
  python3 tools/check_sensors.py --no-teensy  # ODrive(바퀴)만

키: e 힙 FREE(힘 0, 손으로 움직여보기)  h 지금 자세로 힙 고정  d 힙 끄기
    z 지금 IMU 값을 영점 후보로 표시   m 메모 한 줄 로그에 남기기   q 종료
로그: ~/logs/check_*.csv
"""
import sys, os, time, threading, argparse, csv, math, select, termios, tty
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import config as C, model
from teensy_link import Parser, encode_cmd, IX_FLAGS, IX_IMUOK, IX_HBHZ, IX_ENCHZ

class TeensyReader(threading.Thread):
    def __init__(self, port):
        super().__init__(daemon=True)
        import serial
        self.ser = serial.Serial(port, C.TEENSY_BAUD, timeout=0.01)
        self.par = Parser(); self.last = None; self.t_last = 0.0
        self.n = 0; self.rate = 0.0; self.bad_bytes = 0; self.lock = threading.Lock(); self.run_ = True
    def run(self):
        t0 = time.monotonic(); n0 = 0; t_hb = 0
        while self.run_:
            d = self.ser.read(512)
            if d:
                before = len(self.par.buf) + len(d)
                pk = self.par.feed(d)
                if pk:
                    for q in pk:
                        self._check_rate(q)
                    with self.lock:
                        self.last = pk[-1]; self.t_last = time.monotonic(); self.n += len(pk)
            now = time.monotonic()
            if now - t0 >= 1.0:
                self.rate = (self.n - n0) / (now - t0); n0 = self.n; t0 = now
            if now - t_hb > 0.1:
                self.ser.write(encode_cmd('W')); t_hb = now
    # 각도 미분 vs 자이로: 같은 축/부호면 둘이 같이 움직임. 빠르게 흔들 때(> 0.5 rad/s)만 셈
    agree = 0; disagree = 0; _prev = None
    def _check_rate(self, q):
        t = q[0]*1e-6
        ang  = q[2] if C.IMU_AXIS == "pitch" else q[1]
        rate = q[5] if C.IMU_AXIS == "pitch" else q[4]
        if self._prev is not None:
            dt = t - self._prev[0]
            if 0 < dt < 0.05:
                d = (ang - self._prev[1]) / dt
                if abs(rate) > 0.5 and abs(d) > 0.5:
                    if (d > 0) == (rate > 0): self.agree += 1
                    else: self.disagree += 1
        self._prev = (t, ang)
    def cmd(self, c, a=0.0, b=0.0): self.ser.write(encode_cmd(c, a, b))
    def get(self):
        with self.lock: return self.last, self.t_last

class OdriveReader(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        import odrive
        self.odrv = odrive.find_any(timeout=10)
        self.v = None; self.rate = 0.0; self.run_ = True; self.lock = threading.Lock()
    def run(self):
        o = self.odrv; n = 0; t0 = time.monotonic()
        while self.run_:
            a0, a1 = o.axis0, o.axis1
            try:
                v = dict(vbus=o.vbus_voltage,
                         vel=(a0.encoder.vel_estimate, a1.encoder.vel_estimate),
                         pos=(a0.encoder.pos_estimate, a1.encoder.pos_estimate),
                         state=(a0.current_state, a1.current_state),
                         ready=(a0.encoder.is_ready and a0.motor.is_calibrated, a1.encoder.is_ready and a1.motor.is_calibrated),
                         err=(int(a0.error), int(a1.error)))
            except Exception as e:
                v = dict(exc=str(e))
            with self.lock: self.v = v
            n += 1
            if time.monotonic() - t0 >= 1.0:
                self.rate = n / (time.monotonic() - t0); n = 0; t0 = time.monotonic()
            time.sleep(0.02)
    def get(self):
        with self.lock: return self.v

def ok(b): return "\033[32mOK\033[0m " if b else "\033[31mNG\033[0m "

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=C.TEENSY_PORT)
    ap.add_argument("--no-teensy", action="store_true"); ap.add_argument("--no-odrive", action="store_true")
    a = ap.parse_args()
    T = None if a.no_teensy else TeensyReader(a.port)
    O = None if a.no_odrive else OdriveReader()
    for x in (T, O):
        if x: x.start()
    if T:
        T.cmd('K', C.HIP_KP, C.HIP_KD)
        try:                                    # 펌웨어 v3: 바퀴 방향/IMU 부호 등 config 값을 Teensy 에도 보냄 (Teensy 계산값 표시용)
            from main_teensy import default_params, IDX
            for n, v in default_params().items(): T.cmd('P', IDX[n], v)
        except Exception:
            pass
    os.makedirs(os.path.expanduser(C.LOG_DIR), exist_ok=True)
    path = os.path.join(os.path.expanduser(C.LOG_DIR), time.strftime("check_%Y%m%d_%H%M%S.csv"))
    f = open(path, "w", newline=""); w = csv.writer(f)
    w.writerow(["t", "note", "roll_deg", "pitch_deg", "yaw_deg", "gx", "gy", "gz", "hipL", "hipR", "tauL", "tauR",
                "flags", "imu_ok", "th_body", "dphi", "th_off", "th_com", "wheelL_mps", "wheelR_mps"])
    fd = sys.stdin.fileno(); old = termios.tcgetattr(fd); tty.setcbreak(fd)
    zero_hint = None; note = ""; t_log = 0
    TPS = 1.0 / (2*math.pi*C.WHEEL_RADIUS)
    try:
        while True:
            if select.select([sys.stdin], [], [], 0.0)[0]:
                ch = sys.stdin.read(1)
                if ch == "q": break
                if T and ch == "e": T.cmd('E')
                if T and ch == "h": T.cmd('H')
                if T and ch == "d": T.cmd('D')
                if ch == "z": zero_hint = True
                if ch == "m": note = "mark"
            L = []
            L.append("=== TWLR 센서 확인 (바퀴 안 켬) ===   q 종료 | e 힙FREE  h 고정  d 끄기 | z 영점후보  m 메모")
            row = [time.time(), note] + [""]*18; note = ""
            # ---------- Teensy ----------
            if T:
                pk, tl = T.get(); age = time.monotonic() - tl
                L.append(f"[Teensy] 수신 {T.rate:5.0f} Hz (정상 ~250)  {ok(T.rate > 200)}   마지막 수신 {age*1000:5.0f} ms 전 {ok(age < 0.05)}")
                if pk:
                    (tu, r, p, y, gx, gy, gz, hl, hr, vl, vr, tl_, tr, lhz, ius, iev) = pk[:16]
                    fl, imu = int(pk[IX_FLAGS]), pk[IX_IMUOK]
                    hbhz, enchz = pk[IX_HBHZ], pk[IX_ENCHZ]
                    (thc, wc, vm, vc, xm, thr, trim) = pk[16:23]
                    L.append(f"  Teensy 루프 {lhz:5.0f} Hz {ok(lhz > 450)}  IMU 이벤트 {iev:4.0f} Hz {ok(iev > 150)}  IMU 읽기 최대 {ius:5.0f} us {ok(ius < 1500)}")
                    L.append(f"  IMU   ok={imu} {ok(imu)}  roll={math.degrees(r):+7.2f}  pitch={math.degrees(p):+7.2f}  yaw={math.degrees(y):+7.2f} deg")
                    L.append(f"        gyro roll율 gx={gx:+6.2f}  pitch율 gy={gy:+6.2f}  yaw율 gz={gz:+6.2f} rad/s")
                    na = T.agree + T.disagree
                    if na >= 20:
                        frac = T.agree / na
                        L.append(f"  자이로-각도 짝 확인 ({C.IMU_AXIS}): 일치 {100*frac:4.0f}% (샘플 {na}) {ok(frac > 0.9)}"
                                 + ("" if frac > 0.9 else "  ← 축이나 부호가 안 맞음! 펌웨어 v2 맞는지 확인"))
                    else:
                        L.append(f"  자이로-각도 짝 확인: 로봇을 앞뒤로 빠르게 몇 번 흔들어 보세요 (샘플 {na}/20)")
                    L.append(f"  HIP   flags L_en={fl&1} R_en={(fl>>1)&1} hold={(fl>>2)&1} fault={(fl>>3)&1} {ok(not fl&8)}")
                    L.append(f"  [CAN ODrive] 살아있음 L={(fl>>4)&1} R={(fl>>5)&1} {ok(fl&48 == 48)}  closed-loop L={(fl>>6)&1} R={(fl>>7)&1}"
                             f"   바퀴속도(Teensy 계산) {vm:+.2f} m/s  위치 {xm:+.2f} m")
                    L.append(f"              수신: 하트비트 {hbhz:4.0f} Hz (정상 ~20) {ok(hbhz > 10)}  엔코더 응답 {enchz:4.0f} Hz (정상 ~500) {ok(enchz > 300)}")
                    L.append(f"        rel L={hl:+6.2f} R={hr:+6.2f} deg (h 기준, hold 중에만)   tau L={tl_:+5.2f} R={tr:+5.2f} N*m   vel L={vl:+6.1f} R={vr:+6.1f} deg/s")
                    ang = p if C.IMU_AXIS == "pitch" else r
                    th_body = C.IMU_SIGN*math.degrees(ang) - C.IMU_MOUNT_DEG
                    dphi = C.DPHI_SIGN*0.5*(hl + hr) + C.DPHI_AT_H
                    th_off = C.COM_OFF_SIGN*model.theta_offset_deg(dphi)
                    th_com = th_body + th_off + C.COM_TRIM_DEG
                    L.append(f"  계산  th_body={th_body:+6.2f}  dphi={dphi:+6.2f}  th_off={th_off:+5.2f}  => th_CoM={th_com:+6.2f} deg   (config 부호 적용)")
                    if zero_hint:
                        L.append(f"  >> 영점 후보: 지금 몸통이 수평이면 config.IMU_MOUNT_DEG = {C.IMU_SIGN*math.degrees(ang):+.2f}")
                    row[2:20] = [math.degrees(r), math.degrees(p), math.degrees(y), gx, gy, gz, hl, hr, tl_, tr, fl, imu,
                                 th_body, dphi, th_off, th_com, "", ""]
                else:
                    L.append("  아직 패킷 없음 → twlr_bridge 펌웨어가 올라갔는지, 포트가 맞는지, picocom 이 꺼졌는지 확인")
            # ---------- ODrive ----------
            if O:
                v = O.get()
                if not v:
                    L.append("[ODrive] 연결 중...")
                elif "exc" in v:
                    L.append(f"[ODrive] 에러: {v['exc']}")
                else:
                    wl = C.WHEEL_DIR[0]*v['vel'][0]/TPS; wr = C.WHEEL_DIR[1]*v['vel'][1]/TPS
                    L.append(f"[ODrive] 읽기 {O.rate:4.0f} Hz   전압 {v['vbus']:5.2f} V {ok(20 < v['vbus'] < 26)}")
                    L.append(f"  준비(캘리브레이션) L={v['ready'][0]} R={v['ready'][1]} {ok(all(v['ready']))}  상태 {v['state']}  에러 {v['err']} {ok(v['err'] == (0, 0))}")
                    L.append(f"  바퀴 속도 L={wl:+5.2f} R={wr:+5.2f} m/s (전진 +)   위치 L={v['pos'][0]:+8.2f} R={v['pos'][1]:+8.2f} turns")
                    row[18:20] = [wl, wr]
            L.append("")
            L.append("확인 순서: ① 몸통 앞(전진쪽)으로 기울이기 → th_body + ?   ② e 후 다리 손으로 펴기(h 먼저) → dphi + ?")
            L.append("          ③ 바퀴 손으로 앞으로 굴리기 → 속도 + ?   ④ 몸통 수평에서 z → IMU_MOUNT_DEG 후보")
            L.append(f"로그: {path}")
            sys.stdout.write("\033[H\033[J" + "\r\n".join(L) + "\r\n"); sys.stdout.flush()
            if time.time() - t_log > 0.05:
                w.writerow(row); t_log = time.time()
            time.sleep(0.2)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        if T:
            T.cmd('D'); T.run_ = False
        if O: O.run_ = False
        f.close(); print(f"\n종료. 로그: {path}")

if __name__ == "__main__":
    main()
