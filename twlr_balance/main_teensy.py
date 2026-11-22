"""TWLR 밸런싱 - Teensy 제어 버전 (펌웨어 v3).  밸런싱 계산/바퀴 제어/안전은 전부 Teensy 안에서 250 Hz.
이 파이썬은 키보드 -> 명령, 상태 표시, CSV 로그만 한다. (ODrive USB 안 씀 -> odrivetool 켜져 있어도 됨)

  python3 main_teensy.py

키 (엔터 없이, 영문 모드):
  e  힙 켜기(FREE)        h  지금 자세로 힙 고정(= 시작 자세)
  c  지금 자세를 균형점으로 (trim 자동)     [ ]  trim -/+0.2 deg
  b  밸런싱 시작/정지      - =  전체 게인 x0.9 / x1.1
  1 2 3 4  게인 선택 (1 기울기P  2 기울기D  3 속도P  4 속도I)   , .  선택한 게인 x0.9 / x1.1
  i k  전진/후진 +/-0.05 m/s    j l  좌/우 회전    o  속도·회전 0
  x 또는 스페이스  전체 정지 (바퀴 + 힙)      q  종료      ?  도움말
"""
import os, sys, time, struct, threading, csv, select, termios, tty, math, argparse
import config as C
from teensy_link import Parser, encode_cmd, IX_FLAGS, IX_IMUOK, IX_MODE, IX_ERR, IX_THCOM, IX_THREF, IX_VMEAS, IX_VCMD, IX_TRIM, IX_HBHZ, IX_ENCHZ

# 펌웨어 PRM_* 순서와 같아야 함
PRM = ["KTH", "KW", "KPV", "KIV", "GAIN", "IMU_SIGN", "IMU_MOUNT", "COM_SIGN", "DPHI_SIGN", "DPHI_AT_H",
       "DIR_L", "DIR_R", "A_MAX", "V_MAX", "FALL_DEG", "START_DEG", "THREF_MAX", "I_MAX", "R_WHEEL", "TRACK"]
IDX = {n: i for i, n in enumerate(PRM)}
MODE_TXT = {0: "IDLE", 1: "BAL ", 2: "STOP"}
ERR_TXT = ["", "IMU 끊김", "힙 고장/홀드 아님", "넘어짐(30deg)", "ODrive 에러/끊김", "ODrive 준비 안 됨(CAN/엔코더)",
           "시작 기울기 너무 큼(c 먼저)", "PC 끊김", "ODrive closed-loop 진입 실패", "정지 명령"]
GAINS = ["KTH", "KW", "KPV", "KIV"]
GAIN_TXT = ["KPth", "KDth", "KPv", "KIv"]
LOG_COLS = ["t_pc", "t_us", "roll", "pitch", "yaw", "gx", "gy", "gz", "hipL", "hipR", "hipVL", "hipVR", "hipTL", "hipTR",
            "loopHz", "imuUsMax", "imuEvHz", "th_com", "w_com", "v_meas", "v_cmd", "x", "th_ref", "trim_deg", "od_hb_hz", "od_enc_hz",
            "flags", "imu_ok", "bal_mode", "bal_err", "v_ref", "yaw_ref", "gain", "KTH", "KW", "KPV", "KIV"]

def default_params():
    if C.IMU_AXIS != "pitch":
        print("경고: Teensy 제어 버전은 pitch 축만 씀 (config.IMU_AXIS 무시)")
    return {
        "KTH": C.PID_KP_TH, "KW": C.PID_KD_TH, "KPV": C.PID_KP_V, "KIV": C.PID_KI_V, "GAIN": 1.0,
        "IMU_SIGN": C.IMU_SIGN, "IMU_MOUNT": C.IMU_MOUNT_DEG, "COM_SIGN": C.COM_OFF_SIGN,
        "DPHI_SIGN": C.DPHI_SIGN, "DPHI_AT_H": C.DPHI_AT_H, "DIR_L": C.WHEEL_DIR[0], "DIR_R": C.WHEEL_DIR[1],
        "A_MAX": C.A_MAX, "V_MAX": C.V_MAX, "FALL_DEG": C.FALL_DEG, "START_DEG": C.START_MAX_DEG,
        "THREF_MAX": C.PID_THREF_MAX_DEG, "I_MAX": C.PID_I_MAX, "R_WHEEL": C.WHEEL_RADIUS, "TRACK": C.TRACK_WIDTH,
    }

class Link(threading.Thread):
    def __init__(self, port, log_path, params, ui):
        super().__init__(daemon=True)
        import serial
        self.ser = serial.Serial(port, C.TEENSY_BAUD, timeout=0.005)
        self.par = Parser(); self.last = None; self.t_last = 0.0; self.n = 0; self.rate = 0.0
        self.lock = threading.Lock(); self.run_ = True; self.params = params; self.ui = ui
        self.f = open(log_path, "w", newline=""); self.w = csv.writer(self.f); self.w.writerow(LOG_COLS)
    def cmd(self, c, a=0.0, b=0.0):
        with self.lock:
            self.ser.write(encode_cmd(c, a, b))
    def send_params(self):
        for n, v in self.params.items():
            self.cmd('P', IDX[n], v)
    def run(self):
        t_hb = 0.0; t0 = time.monotonic(); n0 = 0; k = 0
        while self.run_:
            try:
                d = self.ser.read(1024)
            except Exception as e:
                self.ui["link_err"] = str(e); time.sleep(0.2); continue
            if d:
                pk = self.par.feed(d)
                now = time.monotonic()
                for p in pk:
                    k += 1
                    self.w.writerow([f"{now:.4f}"] + list(p) + [self.ui["v_ref"], self.ui["yaw_ref"],
                                    self.params["GAIN"]] + [self.params[g] for g in GAINS])
                if pk:
                    self.last = pk[-1]; self.t_last = now; self.n += len(pk)
                    if k % 500 < len(pk): self.f.flush()
            now = time.monotonic()
            if now - t0 >= 1.0:
                self.rate = (self.n - n0) / (now - t0); n0 = self.n; t0 = now
            if now - t_hb > 0.1:
                self.cmd('W'); t_hb = now

def status_line(L, ui):
    p = L.last
    if p is None:
        return "Teensy 패킷 없음 - 펌웨어 v3 맞는지, 포트 맞는지 확인  " + ui.get("link_err", "")
    fl = int(p[IX_FLAGS]); mode = int(p[IX_MODE]); err = int(p[IX_ERR])
    age = time.monotonic() - L.t_last
    sel = ui["sel"]
    gs = " ".join((">" if i == sel else "") + f"{GAIN_TXT[i]}={L.params[g]:.3g}" for i, g in enumerate(GAINS))
    return (f"{MODE_TXT.get(mode, '?')} thCoM={math.degrees(p[IX_THCOM]):+6.2f} thref={math.degrees(p[IX_THREF]):+4.1f} "
            f"v={p[IX_VMEAS]:+.2f}/{p[IX_VCMD]:+.2f} ref={ui['v_ref']:+.2f} yaw={ui['yaw_ref']:+.1f} trim={p[IX_TRIM]:+.1f} "
            f"x{L.params['GAIN']:.2f} [{gs}] hip={fl & 15} od={'L' if fl & 16 else '-'}{'R' if fl & 32 else '-'}"
            f"{'(CL)' if (fl & 192) == 192 else ''} hb={p[IX_HBHZ]:.0f} imu={int(p[IX_IMUOK])} T={p[13]:.0f}Hz rx={L.rate:.0f}Hz"
            f"{' 끊김!' if age > 0.2 else ''}  {ERR_TXT[err] if err < len(ERR_TXT) else err}")

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=C.TEENSY_PORT)
    a = ap.parse_args()
    params = default_params()
    ui = {"v_ref": 0.0, "yaw_ref": 0.0, "sel": 0}
    os.makedirs(os.path.expanduser(C.LOG_DIR), exist_ok=True)
    path = os.path.join(os.path.expanduser(C.LOG_DIR), time.strftime("tbal_%Y%m%d_%H%M%S.csv"))
    L = Link(a.port, path, params, ui); L.start()
    L.cmd('K', C.HIP_KP, C.HIP_KD); L.send_params(); L.cmd('R', C.COM_TRIM_DEG)
    print(f"log -> {path}   ('?' 도움말, 'x' 전체정지, 'q' 종료)")
    fd = sys.stdin.fileno(); old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd); t_st = 0
        while True:
            if select.select([sys.stdin], [], [], 0.05)[0]:
                ch = sys.stdin.read(1); p = L.last
                mode = int(p[IX_MODE]) if p else 0; trim = p[IX_TRIM] if p else 0.0
                if ch in ("x", " "):
                    L.cmd('D'); ui["v_ref"] = ui["yaw_ref"] = 0.0
                elif ch == "e": L.cmd('K', C.HIP_KP, C.HIP_KD); L.cmd('E')
                elif ch == "h": L.cmd('H')
                elif ch == "b":
                    if mode == 1: L.cmd('B', 0)
                    else:
                        ui["v_ref"] = ui["yaw_ref"] = 0.0
                        L.send_params(); L.cmd('V', 0, 0); L.cmd('B', 1)
                elif ch == "c": L.cmd('Z')
                elif ch == "[": L.cmd('R', trim - 0.2)
                elif ch == "]": L.cmd('R', trim + 0.2)
                elif ch in "-=":
                    params["GAIN"] = min(2.0, max(0.2, params["GAIN"] * (0.9 if ch == "-" else 1.1)))
                    L.cmd('P', IDX["GAIN"], params["GAIN"])
                elif ch in "1234": ui["sel"] = int(ch) - 1
                elif ch in ",.":
                    g = GAINS[ui["sel"]]; params[g] *= (0.9 if ch == "," else 1.1); L.cmd('P', IDX[g], params[g])
                elif ch in "ikjlo":
                    if ch == "i": ui["v_ref"] = min(0.5, ui["v_ref"] + 0.05)
                    if ch == "k": ui["v_ref"] = max(-0.5, ui["v_ref"] - 0.05)
                    if ch == "j": ui["yaw_ref"] = min(1.5, ui["yaw_ref"] + 0.2)
                    if ch == "l": ui["yaw_ref"] = max(-1.5, ui["yaw_ref"] - 0.2)
                    if ch == "o": ui["v_ref"] = ui["yaw_ref"] = 0.0
                    L.cmd('V', ui["v_ref"], ui["yaw_ref"])
                elif ch == "?": sys.stdout.write("\r\n" + __doc__.replace("\n", "\r\n") + "\r\n")
                elif ch == "q": break
            if time.monotonic() - t_st > 0.2:
                t_st = time.monotonic()
                sys.stdout.write("\r" + status_line(L, ui)[:230].ljust(230)); sys.stdout.flush()
    finally:
        try:
            L.cmd('D')
        except Exception:
            pass
        time.sleep(0.2); L.run_ = False; time.sleep(0.1)
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        L.f.close()
        print(f"\r\n종료 (바퀴+힙 꺼짐). 로그: {path}")

if __name__ == "__main__":
    main()
