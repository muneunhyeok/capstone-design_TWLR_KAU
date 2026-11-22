"""TWLR 밸런싱 - 실행 파일 (메인 프로세스 = P3: 키보드 + 로그 + 상태 표시)

  python3 main.py            # 실제 로봇 (Teensy + ODrive)
  python3 main.py --sim      # 시뮬레이션 (하드웨어 없이 노트북/Khadas 에서 구조·부호·게인 확인)
  python3 main.py --calibrate  # ODrive 바퀴 캘리브레이션부터 (바퀴가 공중에 떠 있을 것!)

키 (엔터 없이, 영문 모드):
  e  힙 모터 켜기(FREE)        h  지금 자세로 힙 고정(= 시작 자세)
  b  밸런싱 시작/정지           c  지금 자세를 균형점으로 (trim 자동)
  [ ]  균형점 trim -/+0.2 deg  - =  밸런스 게인 x0.9 / x1.1
  i k  전진/후진 속도 +/-0.05 m/s   j l  좌/우 회전   o  속도·회전 0
  p  제어 방식 LQR <-> PID 전환 (밸런싱 꺼져 있을 때만)
  1 2 3 4  PID 게인 선택 (1 기울기P  2 기울기D  3 속도P  4 속도I)   , .  선택한 게인 x0.9 / x1.1
  x 또는 스페이스  전체 비상정지 (바퀴 + 힙)       q  종료
"""
import argparse, os, sys, time, threading, queue, csv, multiprocessing as mp
import config as C, shared
from shared import I, IDLE, BALANCE, FAULT
import control_task

ERR_TXT = ["", "IMU 끊김", "힙 끊김", "넘어짐", "힙 고장/홀드 아님", "ODrive 에러",
           "ODrive 준비 안 됨(캘리브레이션?)", "시작 기울기 너무 큼", "비상정지"]
MODE_TXT = {IDLE: "IDLE", BALANCE: "BAL ", FAULT: "STOP"}
LOG_COLS = ["t", "loop_dt", "mode", "th_body", "w_body", "dphi", "th_off", "th_com", "w_com",
            "v_meas", "v_cmd", "a", "x", "hipL", "hipR", "tauL", "tauR", "v_ref", "yaw_ref", "err", "ctrl_mode", "th_ref"]

def logger(log_q, stop_ev, path):
    with open(path, "w", newline="") as f:
        w = csv.writer(f); w.writerow(LOG_COLS); n = 0
        while not stop_ev.is_set() or not log_q.empty():
            try:
                w.writerow(log_q.get(timeout=0.2)); n += 1
                if n % 200 == 0: f.flush()
            except queue.Empty:
                pass

GAIN_TXT = ["KPth", "KDth", "KPv", "KIv"]

def status(buf, stop_ev):
    while not stop_ev.is_set():
        s = shared.snap(buf)
        if s[I.CTRL_MODE] == shared.PID:
            sel = int(s[I.GAIN_SEL])
            gs = " ".join((">" if k == sel else "") + f"{GAIN_TXT[k]}={s[getattr(I, n)]:.3g}"
                          for k, n in enumerate(shared.PID_GAINS))
            ctl = f"PID[{gs}] thref={s[I.TH_REF]/C.DEG:+.1f}"
        else:
            ctl = "LQR"
        line = (f"{MODE_TXT.get(s[I.MODE], '?')} {ctl} thCoM={s[I.TH_COM]/C.DEG:+6.2f}deg  dphi={s[I.DPHI]:+5.1f}  "
                f"v={s[I.V_MEAS]:+.2f}/{s[I.V_CMD]:+.2f}m/s  ref={s[I.V_REF]:+.2f} yaw={s[I.YAW_REF]:+.2f}  "
                f"trim={s[I.TRIM_DEG]:+.1f} gain x{s[I.GAIN_SCALE]:.2f}  hip={int(s[I.HIP_FLAGS])} "
                f"dtmax={s[I.DT_MAX]*1000:4.1f}ms T={s[I.T_LOOP_HZ]:.0f}Hz  {ERR_TXT[int(s[I.CTRL_ERR])]}")
        sys.stdout.write("\r" + line[:200].ljust(200)); sys.stdout.flush()
        time.sleep(0.25)

def keyboard(buf, hip_cmd_q):
    import termios, tty, select
    fd = sys.stdin.fileno(); old = termios.tcgetattr(fd)
    def hip(c, a=0.0, b=0.0):
        try: hip_cmd_q.put_nowait((c, a, b))
        except queue.Full: pass
    hip('K', C.HIP_KP, C.HIP_KD)
    try:
        tty.setcbreak(fd)
        while not buf[I.QUIT]:
            if not select.select([sys.stdin], [], [], 0.1)[0]:
                continue
            ch = sys.stdin.read(1); s = shared.snap(buf)
            if ch in ("x", " "):
                shared.put(buf, ESTOP=1, REQ_BALANCE=0, V_REF=0, YAW_REF=0); hip('D')
            elif ch == "e": shared.put(buf, ESTOP=0); hip('K', C.HIP_KP, C.HIP_KD); hip('E')
            elif ch == "h": shared.put(buf, ESTOP=0); hip('H')
            elif ch == "b": shared.put(buf, REQ_BALANCE=0 if s[I.MODE] == BALANCE else 1, ESTOP=0)
            elif ch == "c": shared.put(buf, REQ_ZERO_TRIM=1)
            elif ch == "[": shared.put(buf, TRIM_DEG=s[I.TRIM_DEG] - 0.2)
            elif ch == "]": shared.put(buf, TRIM_DEG=s[I.TRIM_DEG] + 0.2)
            elif ch == "-": shared.put(buf, GAIN_SCALE=max(0.2, s[I.GAIN_SCALE]*0.9))
            elif ch == "=": shared.put(buf, GAIN_SCALE=min(2.0, s[I.GAIN_SCALE]*1.1))
            elif ch == "i": shared.put(buf, V_REF=min(0.5, s[I.V_REF] + 0.05))
            elif ch == "k": shared.put(buf, V_REF=max(-0.5, s[I.V_REF] - 0.05))
            elif ch == "j": shared.put(buf, YAW_REF=min(1.5, s[I.YAW_REF] + 0.2))
            elif ch == "l": shared.put(buf, YAW_REF=max(-1.5, s[I.YAW_REF] - 0.2))
            elif ch == "o": shared.put(buf, V_REF=0, YAW_REF=0)
            elif ch == "p" and s[I.MODE] != BALANCE:
                shared.put(buf, CTRL_MODE=shared.LQR if s[I.CTRL_MODE] == shared.PID else shared.PID)
            elif ch in "1234": shared.put(buf, GAIN_SEL=int(ch) - 1)
            elif ch in ",.":
                n = shared.PID_GAINS[int(s[I.GAIN_SEL])]
                shared.put(buf, **{n: s[getattr(I, n)] * (0.9 if ch == "," else 1.1)})
            elif ch == "q": shared.put(buf, ESTOP=1, REQ_BALANCE=0); hip('D'); time.sleep(0.2); shared.put(buf, QUIT=1)
            elif ch == "?": sys.stdout.write("\r\n" + __doc__.replace("\n", "\r\n") + "\r\n")
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sim", action="store_true")
    ap.add_argument("--calibrate", action="store_true")
    ap.add_argument("--port", default=C.TEENSY_PORT)
    args = ap.parse_args()

    buf, hip_cmd_q, log_q = shared.make()
    procs = []
    if not args.sim:
        import teensy_link
        procs.append(mp.Process(target=teensy_link.run, args=(buf, hip_cmd_q, args.port, C.TEENSY_BAUD), daemon=True))
    procs.append(mp.Process(target=control_task.run, args=(buf, hip_cmd_q, log_q, args.sim, args.calibrate), daemon=True))
    for p in procs: p.start()

    os.makedirs(os.path.expanduser(C.LOG_DIR), exist_ok=True)
    path = os.path.join(os.path.expanduser(C.LOG_DIR), time.strftime("bal_%Y%m%d_%H%M%S.csv"))
    stop_ev = threading.Event()
    threads = [threading.Thread(target=logger, args=(log_q, stop_ev, path), daemon=True),
               threading.Thread(target=status, args=(buf, stop_ev), daemon=True)]
    for t in threads: t.start()
    print(f"log -> {path}   ('?' 도움말, 'x' 비상정지, 'q' 종료)")
    try:
        keyboard(buf, hip_cmd_q)
    except KeyboardInterrupt:
        pass
    finally:
        shared.put(buf, ESTOP=1, REQ_BALANCE=0, QUIT=1)
        for p in procs: p.join(timeout=2)
        stop_ev.set()
        for t in threads: t.join(timeout=2)
        print(f"\r\n종료. 로그: {path}")

if __name__ == "__main__":
    main()
