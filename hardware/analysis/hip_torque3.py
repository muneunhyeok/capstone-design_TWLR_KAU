# -*- coding: utf-8 -*-
"""4절 링크 폐쇄해 + 가상일 원리로 고관절 필요토크.  mass_cog.py 를 먼저 실행 (parts.json).
   액추에이터 정격/피크는 mass_cog.py 의 TWLR_ACT 선택을 따름 (기본 DM4340P)."""
import math, json, os
HERE=os.path.dirname(os.path.abspath(__file__))
ACTUATORS={
    "AK45":    dict(name="CubeMars AK45-36 V3.0 KV80", rated=8.0, peak=24.0),
    "DM4340P": dict(name="Damiao DM-J4340P-2EC (24V, 40:1)", rated=9.0, peak=27.0),
}
ACT_KEY=os.environ.get("TWLR_ACT","DM4340P"); ACT=ACTUATORS[ACT_KEY]
T_RATED=ACT["rated"]; T_PEAK=ACT["peak"]
g=9.80665
rows=json.load(open(os.path.join(HERE,"parts.json")))
HIP=(0.,0.); A=(114.,49.); KNEE0=(174.,-164.); B0=(228.5,-128.6); WHL0=(4.,-336.); RW=86.
sub=lambda p,q:(p[0]-q[0],p[1]-q[1]); nrm=lambda v:math.hypot(*v); ang=lambda v:math.atan2(v[1],v[0])
def rot(v,a): c,s=math.cos(a),math.sin(a); return (c*v[0]-s*v[1], s*v[0]+c*v[1])
L1=nrm(sub(KNEE0,HIP)); L2=nrm(sub(B0,KNEE0)); L3=nrm(sub(B0,A))
P1,P2,P3=ang(sub(KNEE0,HIP)),ang(sub(B0,KNEE0)),ang(sub(B0,A))

def solve(dth, prev):
    ph1=P1+dth; knee=(L1*math.cos(ph1), L1*math.sin(ph1))
    d=sub(A,knee); D=nrm(d)
    if D>L2+L3 or D<abs(L2-L3): return None
    a=(L2*L2-L3*L3+D*D)/(2*D); h2=L2*L2-a*a
    if h2<0: return None
    h=math.sqrt(h2); ux,uy=d[0]/D,d[1]/D; mx,my=knee[0]+a*ux,knee[1]+a*uy
    cs=[(mx+h*uy,my-h*ux),(mx-h*uy,my+h*ux)]
    B=min(cs,key=lambda p:abs(((ang(sub(p,knee))-prev+math.pi)%(2*math.pi))-math.pi))
    ph2=ang(sub(B,knee)); ph3=ang(sub(B,A))
    wv=rot(sub(WHL0,KNEE0), ph2-P2)
    return dict(ph1=ph1,ph2=ph2,ph3=ph3,knee=knee,B=B,wheel=(knee[0]+wv[0],knee[1]+wv[1]))

def track(dth):
    """0 -> dth 로 연속 추적하여 분기 오류 방지"""
    prev=P2; st=solve(0.0,prev)
    n=max(1,int(abs(math.degrees(dth))/0.25))
    for i in range(1,n+1):
        st2=solve(dth*i/n, st['ph2'])
        if st2 is None: return None
        st=st2
    return st

def energy(st):
    Hh=RW-st['wheel'][1]; d1=st['ph1']-P1; d2=st['ph2']-P2; d3=st['ph3']-P3
    E=0.; M=0.; Mz=0.
    for r in rows:
        y,z=r['c'][1],r['c'][2]; m=r['m']
        if r['g']=="BODY": p=(y,z)
        elif r['g']=="THIGH": p=rot((y,z),d1)
        elif r['g']=="SHIN":
            v=rot(sub((y,z),KNEE0),d2); p=(st['knee'][0]+v[0], st['knee'][1]+v[1])
        else:
            v=rot(sub((y,z),A),d3); p=(A[0]+v[0], A[1]+v[1])
        zz=Hh+p[1]; E+=m*zz; M+=m; Mz+=m*zz
    return E*g/1e6, M, Mz/M, Hh          # J, g, mm, mm

hstep=math.radians(0.2)
def tau(dd):
    a=track(dd+hstep); b=track(dd-hstep)
    if a is None or b is None: return None
    return (energy(a)[0]-energy(b)[0])/(2*hstep)/2.0     # 다리 1개당 N·m

print("="*104)
print(f" {ACT['name']} 고관절 정역학  —  다리 1개당 필요 토크와 모터 여유")
print(f" 정격 {T_RATED:g} N·m / 피크 {T_PEAK:g} N·m (출력축).  허용듀티 = (정격/필요)² — 전류∝토크 가정의 I² 발열 기준")
print("="*104)
print(f"{'접힘Δθ':>8s}{'힙높이':>9s}{'CoG높이':>9s}{'무릎각':>8s}{'필요토크':>10s}"
      f"{'정격 대비':>10s}{'피크 대비':>10s}{'허용듀티':>10s}{'판정':>16s}")
print(f"{'[deg]':>8s}{'[mm]':>9s}{'[mm]':>9s}{'[deg]':>8s}{'[N·m]':>10s}{'':>10s}{'':>10s}{'(I²기준)':>10s}{'':>16s}")
print("-"*104)
data=[]
for d in [-30,-25,-22,-20,-15,-10,-5,0,5,10,15,20,22,25,27,29,30,31,32]:
    dd=math.radians(d); st=track(dd)
    if st is None: print(f"{d:8.0f}   (4절링크 해 없음 - 기구학 한계)"); continue
    t=tau(dd)
    if t is None: print(f"{d:8.0f}   (한계 근처)"); continue
    t=abs(t); E,M,cz,Hh=energy(st)
    duty=min(1.0,(T_RATED/t)**2)
    kn=math.degrees(st['ph2']-st['ph1'])
    verdict = "연속 가능" if t<=T_RATED else ("피크내 (단시간)" if t<=T_PEAK else "불가")
    print(f"{d:8.0f}{Hh:9.0f}{cz:9.0f}{kn:8.1f}{t:10.2f}{t/T_RATED*100:9.0f}%{t/T_PEAK*100:9.0f}%{duty*100:9.0f}%{verdict:>16s}")
    data.append((d,Hh,cz,kn,t,duty))

json.dump(data, open(os.path.join(HERE,"torque2.json"),"w"))

print()
def find(lo,hi,target):
    for _ in range(60):
        mid=(lo+hi)/2
        st=track(math.radians(mid)); t=tau(math.radians(mid))
        if st is None or t is None: return None
        if abs(t)<target: lo=mid
        else: hi=mid
    return (lo+hi)/2
def bracket(target):
    """필요토크가 target 을 넘는 첫 1° 구간"""
    prev=None
    for d in range(-27,37):
        st=track(math.radians(d)); t=tau(math.radians(d))
        if st is None or t is None: continue
        if abs(t)>=target and prev is not None: return find(prev,d,target)
        prev=d
    return None
cR=bracket(T_RATED); cP=bracket(T_PEAK)
for nm,c in ((f"정격 {T_RATED:g} N·m 초과 시작", cR), (f"피크 {T_PEAK:g} N·m 도달", cP)):
    if c is None: print(f"  {nm}: 가동범위 내 없음"); continue
    st=track(math.radians(c)); E,M,cz,Hh=energy(st)
    print(f"  {nm}:  Δθ = {c:+.2f}°  →  힙높이 {Hh:.0f} mm, CoG높이 {cz:.0f} mm, 무릎각 {math.degrees(st['ph2']-st['ph1']):.1f}°, 필요토크 {abs(tau(math.radians(c))):.2f} N·m")
lo,hi=-30.0,-25.0
for _ in range(60):
    mid=(lo+hi)/2
    if track(math.radians(mid)) is None: lo=mid
    else: hi=mid
st=track(math.radians(hi)); print(f"  기구학 신전 한계: Δθ = {hi:+.2f}°  (힙높이 {RW-st['wheel'][1]:.0f} mm)")
lo,hi=32.0,60.0
for _ in range(60):
    mid=(lo+hi)/2
    if track(math.radians(mid)) is None: hi=mid
    else: lo=mid
st=track(math.radians(lo)); print(f"  기구학 접힘 한계: Δθ = {lo:+.2f}°  (힙높이 {RW-st['wheel'][1]:.0f} mm)")

# ---- 휠모터(WA172E) 밸런싱 여유 : 새 질량 기준 ------------------------
st=track(0.0); E,M,cz,Hh=energy(st)
Mkg=M/1000.0
Lpend = cz - RW     # 휠축 위 CoG 높이 (mm)
print()
print("="*80)
print(" WA172E 휠모터 자세복원 토크 (직립자세 Δθ=0, 총질량 %.2f kg, 휠축 위 CoG %.0f mm)"%(Mkg,Lpend))
print("="*80)
print(f"{'기울기[deg]':>12s}{'휠1개 필요토크[N·m]':>22s}{'정격5 대비':>12s}{'스톨13 대비':>13s}")
for a in (5,10,15,20,25,30):
    T = Mkg*9.80665*(Lpend/1000.0)*math.sin(math.radians(a))/2.0
    print(f"{a:12d}{T:22.2f}{T/5*100:11.0f}%{T/13*100:12.0f}%")
