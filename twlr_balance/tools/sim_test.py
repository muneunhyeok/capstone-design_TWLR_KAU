"""키보드 없이 --sim 동작 확인: 밸런싱 시작 → 5초 유지 → 전진 0.2 m/s → 정지. 결과 요약 출력."""
import sys, os, time, multiprocessing as mp, queue
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import shared, control_task, config as C
from shared import I

def main():
    buf, hq, lq = shared.make()
    if '--pid' in sys.argv: buf[I.CTRL_MODE] = shared.PID
    print('mode:', 'PID' if buf[I.CTRL_MODE] == shared.PID else 'LQR')
    p = mp.Process(target=control_task.run, args=(buf, hq, lq, True, False)); p.start()
    time.sleep(0.3); shared.put(buf, REQ_BALANCE=1)
    rows = []
    def drain(sec):
        t_end = time.time() + sec
        while time.time() < t_end:
            try: rows.append(lq.get(timeout=0.1))
            except queue.Empty: pass
    drain(5); shared.put(buf, V_REF=0.2); drain(5); shared.put(buf, V_REF=0.0); drain(3)
    s = shared.snap(buf); shared.put(buf, QUIT=1); p.join()
    bal = [r for r in rows if r[2] == 1.0]
    th = [abs(r[7])/C.DEG for r in bal[-200:]]
    print("rows", len(rows), "balance rows", len(bal), "final mode", s[I.MODE], "err", s[I.CTRL_ERR])
    print("max |thCoM| last 2s: %.2f deg" % (max(th) if th else -1))
    v = [r[9] for r in rows if 6 < (r[0]-rows[0][0]) < 10]
    print("mean v during v_ref=0.2: %.3f" % (sum(v)/len(v) if v else 0))
    dts = [r[1] for r in rows[10:]]
    print("loop dt mean %.2f ms max %.2f ms" % (1000*sum(dts)/len(dts), 1000*max(dts)))
    print("trim needed (model err 1 deg) -> final th_com %.2f deg" % (rows[-1][7]/C.DEG))

if __name__ == "__main__":
    main()
