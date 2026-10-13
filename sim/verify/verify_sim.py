#!/usr/bin/env python3
"""TWLR 시뮬레이션 패키지(sim/) 검증 스크립트 — Gazebo 없이 URDF 숫자와 제어기 코드만으로 확인한다.

  python3 sim/verify/verify_sim.py            # 결과를 화면에 출력
  python3 sim/verify/verify_sim.py > sim/verify/results/sim-verification.txt

필요: pip install xacro numpy
확인 항목
  1. xacro 변환(다리 고정/이동), 관성 텐서 물리 타당성, 총질량·무게중심·균형각
  2. STL 단위·원점
  3. 스폰 높이, 4절 링크 길이·닫힘
  4. 제어기(balance_core.py) 를 URDF 질량값으로 만든 평면 바퀴-역진자 모델에 붙여 시뮬레이션
  5. 지연·게인·질량·토크 한계 여유 탐색
한계: 접촉(마찰·미끄러짐), 브리지 지연 지터, 센서 노이즈, 바퀴·지면 접촉 모델은 없다. 실제 Gazebo 구동 결과가 아니다.
"""
import math, os, struct, sys, tempfile
import numpy as np
import xml.etree.ElementTree as ET

sys.dont_write_bytecode = True
HERE = os.path.dirname(os.path.abspath(__file__))
DESC = os.path.normpath(os.path.join(HERE, '..', 'twlr_description'))
CTRL = os.path.normpath(os.path.join(HERE, '..', 'twlr_control', 'twlr_control'))
sys.path.insert(0, CTRL)
from balance_core import BalanceCore, DEFAULTS      # noqa: E402

import xacro                                          # noqa: E402

R_WHEEL = 0.086


def vec(s):
    return np.array([float(x) for x in s.split()])


def build_urdf(acrylic, legs_fixed):
    """$(find twlr_description) 을 실제 경로로 바꾼 임시 사본으로 xacro 변환."""
    src = open(os.path.join(DESC, 'urdf', 'twlr.urdf.xacro'), encoding='utf-8').read()
    src = src.replace('$(find twlr_description)', DESC)
    with tempfile.NamedTemporaryFile('w', suffix='.xacro', delete=False, encoding='utf-8') as f:
        f.write(src)
        tmp = f.name
    try:
        doc = xacro.process_file(tmp, mappings={'acrylic_mass': str(acrylic),
                                                'legs_fixed': 'true' if legs_fixed else 'false'})
        return doc.toxml()
    finally:
        os.unlink(tmp)


class Model:
    """legs_fixed 모델의 질량·위치 (모든 조인트 rpy = 0 이므로 이동만 누적)."""
    def __init__(self, acrylic):
        self.root = ET.fromstring(build_urdf(acrylic, True))
        self.parent = {j.find('child').get('link'): (j.find('parent').get('link'), vec(j.find('origin').get('xyz')))
                       for j in self.root.findall('joint')}
        self.axle = self.pos('wheel_L')
        self.bodies, self.Mw, self.Iw, self.bad = [], 0.0, 0.0, []
        for l in self.root.findall('link'):
            i = l.find('inertial')
            if i is None:
                continue
            n = l.get('name')
            m = float(i.find('mass').get('value'))
            c = self.pos(n) + vec(i.find('origin').get('xyz'))
            a = i.find('inertia').attrib
            I = np.array([[a['ixx'], a['ixy'], a['ixz']], [a['ixy'], a['iyy'], a['iyz']], [a['ixz'], a['iyz'], a['izz']]], float)
            w = np.linalg.eigvalsh(I)
            if w[0] <= 0 or w[0] + w[1] < w[2] * (1 - 1e-6):
                self.bad.append(n)
            if n.startswith('wheel'):
                self.Mw += m
                self.Iw += float(a['iyy'])
            self.bodies.append((n, m, c, float(a['iyy']), n.startswith('wheel')))
        nb = [b for b in self.bodies if not b[4]]
        self.mb = sum(b[1] for b in nb)
        self.com_b = sum(b[1] * b[2] for b in nb) / self.mb
        self.M = sum(b[1] for b in self.bodies)
        self.com = sum(b[1] * b[2] for b in self.bodies) / self.M
        rel = self.com_b - self.axle
        self.lx, self.lz = rel[0], rel[2]
        self.Ib = sum(b[3] + b[1] * ((b[2][0] - self.com_b[0]) ** 2 + (b[2][2] - self.com_b[2]) ** 2) for b in nb)

    def pos(self, l):
        p = np.zeros(3)
        while l in self.parent:
            pl, o = self.parent[l]
            p = p + o
            l = pl
        return p

    def plant(self):
        return dict(mb=self.mb, lx=self.lx, lz=self.lz, Ib=self.Ib, Mw=self.Mw, Iw=self.Iw, r=R_WHEEL)

    def eq_pitch_deg(self):
        return math.degrees(math.atan2(-self.lx, self.lz))


# ---------------------------------------------------------------- plant / simulation
def deriv(s, tau, P, g=9.81):
    """x = 전진 위치, th = pitch(+ 앞으로 숙임). 바퀴 회전 관성은 질량 Iw/r^2 로 환산. tau = 두 바퀴 토크 합."""
    x, th, xd, thd = s
    r, mb, lx, lz = P['r'], P['mb'], P['lx'], P['lz']
    X = lx * math.cos(th) + lz * math.sin(th)
    Z = -lx * math.sin(th) + lz * math.cos(th)
    A = P['Mw'] + P['Iw'] / r ** 2 + mb
    B = mb * Z
    D = mb * (lx * lx + lz * lz) + P['Ib']
    rhs1 = tau / r + mb * X * thd ** 2
    rhs2 = mb * g * X - tau
    det = A * D - B * B
    return np.array([xd, thd, (rhs1 * D - B * rhs2) / det, (A * rhs2 - B * rhs1) / det])


def simulate(P, offset_deg, th0_deg, T=12.0, vcmd=None, push=None, params=None, delay_ms=0.0):
    p = dict(DEFAULTS)
    p['pitch_offset_deg'] = offset_deg
    if params:
        p.update(params)
    core = BalanceCore(p)
    s = np.array([0.0, math.radians(th0_deg), 0.0, 0.0])
    dt, n = 0.0005, 5                       # 적분 0.5 ms, 제어 2.5 ms (IMU 400 Hz)
    nd = int(round(delay_ms / (dt * 1000)))
    queue = [0.0] * nd
    tau_cmd = tau = 0.0
    rec = []
    for k in range(int(T / dt)):
        t = k * dt
        if k % n == 0:
            w = s[2] / P['r'] - s[3]
            o = core.step(n * dt, s[1], s[3], 0.0, w, w, vcmd(t) if vcmd else 0.0, 0.0)
            tau_cmd = o['tau_l'] + o['tau_r']
        queue.append(tau_cmd)
        tau = queue.pop(0)
        if push and abs(t - push[0]) < dt / 2:
            s[2] += push[1]
        k1 = deriv(s, tau, P); k2 = deriv(s + .5 * dt * k1, tau, P)
        k3 = deriv(s + .5 * dt * k2, tau, P); k4 = deriv(s + dt * k3, tau, P)
        s = s + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        if core.fallen or abs(s[1]) > math.radians(60):
            return dict(fell=True, t_fell=t, rec=np.array(rec))
        if k % 40 == 0:
            rec.append((t, math.degrees(s[1]), s[2], s[0], tau / 2))
    return dict(fell=False, rec=np.array(rec), end=s)


def settled(res, vtol=0.01, ptol=0.1):
    if res['fell']:
        return 'FELL(%.1fs)' % res['t_fell']
    r = res['rec']
    tail = r[int(len(r) * 0.8):]
    ok = abs(r[-1, 2]) < vtol and np.ptp(tail[:, 1]) < ptol
    return 'ok' if ok else 'NOT-SETTLED(v=%+.2f,pp=%.1fdeg)' % (r[-1, 2], np.ptp(tail[:, 1]))


# ---------------------------------------------------------------- checks
def stl_bbox(path):
    d = open(path, 'rb').read()
    n = struct.unpack('<I', d[80:84])[0]
    a = np.frombuffer(d[84:84 + 50 * n], dtype=np.dtype([('n', '<3f4'), ('v', '<9f4'), ('a', '<u2')]))
    v = a['v'].reshape(-1, 3).astype(float)
    return v.min(0), v.max(0), n


def main():
    print('=' * 100)
    print(' TWLR sim/ 검증 결과  (Gazebo 없이: URDF 숫자 + 제어기 코드 + 평면 바퀴-역진자 모델)')
    print('=' * 100)

    print('\n[1] URDF — xacro 변환, 관성 타당성, 질량·무게중심·균형각')
    models = {}
    for acr in (0.515, 0.001):
        m = Model(acr)
        models[acr] = m
        rel = m.com_b - m.axle
        print(f'  acrylic_mass={acr:<6} 총질량 {m.M:.3f} kg | 비정상 관성 링크: {m.bad or "없음"}')
        print(f'      몸통+다리 무게중심(바퀴축 기준) dx={rel[0]*1000:+.1f} mm, dz={rel[2]*1000:+.1f} mm'
              f' → 균형 pitch {m.eq_pitch_deg():.2f}° (+ = 앞으로 숙임)')
        print(f'      전체 무게중심 지면 높이 {(m.com[2] - m.axle[2] + R_WHEEL)*1000:.1f} mm'
              f' | 몸통+다리 {m.mb:.3f} kg, 바퀴 {m.Mw:.3f} kg')
    ET.fromstring(build_urdf(0.515, False))
    print('  legs_fixed:=false 변환 OK, closure 조인트 포함:', 'closure_L_joint' in build_urdf(0.515, False))

    print('\n[2] 메시 단위·원점 (m 이어야 하고, 바퀴는 지름 0.172 에 원점이 중심)')
    for f in sorted(os.listdir(os.path.join(DESC, 'meshes'))):
        lo, hi, n = stl_bbox(os.path.join(DESC, 'meshes', f))
        print(f'  {f:16s} 삼각형 {n:6d}  크기 {np.round(hi - lo, 3)}  min {np.round(lo, 3)}  max {np.round(hi, 3)}')

    print('\n[3] 기하 — 스폰 높이, 4절 링크')
    m = models[0.515]
    bottom = -(m.axle[2]) + R_WHEEL
    print(f'  휠 바닥 = base_link 아래 {bottom:.3f} m, sim.launch.py 스폰 z = 0.430 → 낙하 {0.430 - bottom:.3f} m')
    th = 0.101
    wx = m.axle[0] * math.cos(th) + m.axle[2] * math.sin(th)
    wz = -m.axle[0] * math.sin(th) + m.axle[2] * math.cos(th)
    print(f'  spawn_pitch 0.101 rad 일 때 낙하 {0.430 + wz - R_WHEEL:.3f} m')
    hip = np.array([0, 0.167, 0]); knee = hip + [-0.174, 0.0205, -0.164]
    cpl = np.array([-0.114, 0.1666, 0.049]); clo = knee + [-0.0545, 0.0058, 0.0354]
    pl = lambda a, b: math.hypot(a[0] - b[0], a[2] - b[2])
    print('  측면(xz) 평면 링크 길이 [mm]  — README 값과 비교')
    print(f'    thigh   hip→knee            {pl(knee, hip)*1000:6.1f}   (239.1)')
    print(f'    coupler 피벗→closure        {pl(clo, cpl)*1000:6.1f}   (211.3)')
    print(f'    shin    knee→휠축           {math.hypot(0.17, 0.172)*1000:6.1f}   (241.8)')
    print(f'    몸통    hip→coupler 피벗    {pl(cpl, hip)*1000:6.1f}   (124.1)')
    print(f'    shin 내 knee→closure        {math.hypot(0.0545, 0.0354)*1000:6.1f}')
    print(f'  coupler↔shin closure 점 y 어긋남 (축 방향, 허용): {abs(clo[1] - cpl[1])*1000:.1f} mm')

    P = models[0.515].plant()
    P_off = models[0.001].plant()

    print('\n[4] 제어기 시나리오 (balance_core.py 원본 + 기본 게인, 평면 모델)')
    sc = [
        ('A 균형각에서 시작', P, 5.8, 5.79, {}),
        ('B 직립(0°)에서 시작', P, 5.8, 0.0, {}),
        ('C −6°에서 시작', P, 5.8, -6.0, {}),
        ('D +15°에서 시작', P, 5.8, 15.0, {}),
        ('E 0.25 m/s 밀침 (2 s)', P, 5.8, 5.79, dict(push=(2.0, 0.25))),
        ('F 0.6 m/s 밀침 (2 s)', P, 5.8, 5.79, dict(push=(2.0, 0.6))),
        ('G v_cmd 0.4 m/s, 2~6 s', P, 5.8, 5.79, dict(vcmd=lambda t: 0.4 if 2 < t < 6 else 0.0)),
        ('H 아크릴 제거 + offset 5.8 (README 명령 그대로)', P_off, 5.8, 10.1, {}),
        ('I 아크릴 제거 + offset 8.2', P_off, 8.2, 8.2, {}),
    ]
    for name, PP, off, th0, kw in sc:
        r = simulate(PP, off, th0, T=14, **kw)
        if r['fell']:
            print(f'  {name:44s} FELL at {r["t_fell"]:.1f}s'); continue
        rr = r['rec']
        print(f'  {name:44s} {settled(r):6s} 최종 pitch {rr[-1,1]:6.2f}° x {rr[-1,3]:+.2f} m | 최대 |토크/휠| {np.abs(rr[:,4]).max():.2f} N·m'
              f' | 최대 pitch 이탈 {np.abs(rr[:,1] - rr[-1,1]).max():.1f}°')

    print('\n[5] 여유 탐색 (0.25 m/s 밀침 후 20 s 내 정착 여부 · ok / FELL / NOT-SETTLED)')
    def sweep(title, values, make):
        out = []
        for v in values:
            P2, kw = make(v)
            out.append(f'{v}:{settled(simulate(P2, 5.8, 5.79, T=20, push=(2.0, 0.25), **kw))}')
        print(f'  {title}\n     ' + '  '.join(out))
    sweep('센서→토크 지연 [ms]', (0, 10, 20, 30, 40, 50, 60), lambda v: (P, dict(delay_ms=v)))
    sweep('kp_pitch (kd 4)', (12, 15, 18, 20, 30, 45, 60, 80), lambda v: (P, dict(params=dict(kp_pitch=v))))
    sweep('kd_pitch (kp 30)', (1, 2, 3, 4, 6, 8, 12, 16), lambda v: (P, dict(params=dict(kd_pitch=v))))
    sweep('무게중심 높이 배율 (offset 5.8 유지)', (0.7, 0.8, 1.0, 1.2, 1.4),
          lambda v: (dict(P, lz=P['lz'] * v), {}))
    sweep('몸통 질량 배율 (관성 비례)', (0.7, 1.0, 1.3, 1.5, 2.0),
          lambda v: (dict(P, mb=P['mb'] * v, Ib=P['Ib'] * v), {}))
    sweep('휠당 토크 한계 [N·m]', (0.3, 0.5, 0.8, 1.0, 2.0, 10.0),
          lambda v: (P, dict(params=dict(max_wheel_torque=v))))
    print('\n[6] 선형화 — 균형점 부근 열린 루프 (URDF 값 기준) vs 저장소 docs/electronics/02 의 LQR 모델')
    for acr, mm in models.items():
        PP = mm.plant()
        th_eq = math.radians(mm.eq_pitch_deg())
        s0 = np.array([0.0, th_eq, 0.0, 0.0]); e = 1e-6
        A = np.zeros((4, 4))
        for i in range(4):
            d = np.zeros(4); d[i] = e
            A[:, i] = (deriv(s0 + d, 0.0, PP) - deriv(s0 - d, 0.0, PP)) / (2 * e)
        Bv = (deriv(s0, e, PP) - deriv(s0, -e, PP)) / (2 * e)
        pole = max(np.linalg.eigvals(A).real)
        print(f'  acrylic={acr:<6} 불안정 극점 ±{pole:.2f} rad/s (기울기 2배 시간 {math.log(2)/pole*1000:.0f} ms)'
              f' | 입력 이득: 가속도 {Bv[2]:+.2f} m/s²/(N·m), 각가속도 {Bv[3]:+.2f} rad/s²/(N·m)'
              f' | 몸통 관성 Ib(무게중심 기준) {mm.Ib:.4f} kg·m²')
    print('  저장소 LQR 모델(추정 I_w 0.0085, I_b 0.07): 불안정 극점 ±7.59 rad/s, a = +2.34τ, α = −8.33τ  → 값이 다르므로 K 재계산 여부 판단 필요')

    print('\n주의: 위는 이상적 평면 모델 결과다. 실제 Gazebo/실물에서는 접촉·백래시·지연 지터로 여유가 더 작을 수 있다.')


if __name__ == '__main__':
    main()
