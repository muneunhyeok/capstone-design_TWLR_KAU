# -*- coding: utf-8 -*-
"""4절 링크 기구학 셀프체크: (1) 루프 폐쇄 잔차 (2) 링크 길이 보존 (3) 토크 두 방법 비교
   방법 A = 위치에너지 수치미분 (hip_torque3.py 와 동일),  방법 B = 속도해석(야코비안) 해석해
   + 전달각, 바퀴 수평 이동, Grashof 판정.  사용: python3 mass_cog.py && python3 kin_selfcheck.py"""
import sys, os, math, json
AN = os.path.dirname(os.path.abspath(__file__))
sys.argv = [sys.argv[0]]
sys.path.insert(0, AN)
import io, contextlib
with contextlib.redirect_stdout(io.StringIO()):
    import hip_torque3 as H
g = 9.80665
rows = H.rows
L0 = math.hypot(*H.A)  # 몸통(HIP-A)
links = dict(ground=L0, thigh=H.L1, coupler_KB=H.L2, rocker_AB=H.L3)
s = sorted(links.values()); grashof = s[0] + s[3] <= s[1] + s[2]

def part_pos(r, st):
    y, z = r['c'][1], r['c'][2]
    d1 = st['ph1'] - H.P1; d2 = st['ph2'] - H.P2; d3 = st['ph3'] - H.P3
    if r['g'] == "BODY": return (y, z), 'B'
    if r['g'] == "THIGH": return H.rot((y, z), d1), 'T'
    if r['g'] == "SHIN":
        v = H.rot(H.sub((y, z), H.KNEE0), d2); return (st['knee'][0] + v[0], st['knee'][1] + v[1]), 'S'
    v = H.rot(H.sub((y, z), H.A), d3); return (H.A[0] + v[0], H.A[1] + v[1]), 'C'

def tau_jacobian(st):
    p1, p2, p3 = st['ph1'], st['ph2'], st['ph3']; w1 = 1.0
    # i L1 w1 e^{ip1} + i L2 w2 e^{ip2} = i L3 w3 e^{ip3}
    a11, a12 = -H.L2 * math.sin(p2), H.L3 * math.sin(p3)
    a21, a22 = H.L2 * math.cos(p2), -H.L3 * math.cos(p3)
    b1, b2 = H.L1 * w1 * math.sin(p1), -H.L1 * w1 * math.cos(p1)
    det = a11 * a22 - a12 * a21
    w2 = (b1 * a22 - a12 * b2) / det; w3 = (a11 * b2 - b1 * a21) / det
    knee = st['knee']; vk = (-w1 * knee[1], w1 * knee[0])
    def vz(p, k):
        if k == 'B': return 0.0
        if k == 'T': return w1 * p[0]
        if k == 'S': return vk[1] + w2 * (p[0] - knee[0])
        return w3 * (p[0] - H.A[0])
    vzw = vz(st['wheel'], 'S')          # 바퀴는 지면에 고정 → 몸통이 -vzw 만큼 오르내림
    dE = sum(r['m'] * (vz(part_pos(r, st)[0], part_pos(r, st)[1]) - vzw) for r in rows)
    return dE * g / 1e6 / 2.0, w2, w3   # 다리 1개당 N·m (mm,g → m,kg)

out = []; worst_close = 0; worst_len = 0
for d10 in range(-270, 361, 5):
    d = d10 / 10; st = H.track(math.radians(d))
    if st is None: continue
    B, k = st['B'], st['knee']
    close = math.hypot(B[0] - (H.A[0] + H.L3 * math.cos(st['ph3'])), B[1] - (H.A[1] + H.L3 * math.sin(st['ph3'])))
    dl = max(abs(math.hypot(*k) - H.L1), abs(math.hypot(B[0]-k[0], B[1]-k[1]) - H.L2), abs(math.hypot(B[0]-H.A[0], B[1]-H.A[1]) - H.L3))
    worst_close = max(worst_close, close); worst_len = max(worst_len, dl)
    tA = H.tau(math.radians(d)); tB, w2, w3 = tau_jacobian(st)
    v_c = (k[0]-B[0], k[1]-B[1]); v_r = (H.A[0]-B[0], H.A[1]-B[1])
    mu = math.degrees(math.acos((v_c[0]*v_r[0]+v_c[1]*v_r[1]) / (H.L2*H.L3)))
    E, M, cz, Hh = H.energy(st)
    out.append(dict(d=d, hip_h=Hh, wheel_y=st['wheel'][0], knee=math.degrees(st['ph2']-st['ph1']),
                    mu=mu, tA=None if tA is None else abs(tA), tB=abs(tB), cog=cz, w3=w3))
diff = max(abs(o['tA'] - o['tB']) for o in out if o['tA'] is not None)
res = dict(links=links, grashof=grashof, shortest=min(links, key=links.get), worst_close_mm=worst_close,
           worst_len_mm=worst_len, max_tau_diff=diff, mass_kg=H.energy(H.track(0))[1]/1000, rows=out)
json.dump(res, open(os.path.join(AN, 'kin_selfcheck.json'), 'w'), ensure_ascii=False, indent=1)
print({k: v for k, v in res.items() if k != 'rows'})
for o in out:
    if o['d'] in (-27, -25, -20, -10, 0, 10, 20, 25, 30, 36) or abs(o['d']) % 5 == 0 and o['d'] % 10 == 0:
        print(f"{o['d']:6.1f} hip_h {o['hip_h']:6.1f}  wheel_y {o['wheel_y']:6.1f} knee {o['knee']:6.1f} mu {o['mu']:5.1f}  tA {o['tA'] if o['tA'] is None else round(o['tA'],3)} tB {o['tB']:.3f}")
