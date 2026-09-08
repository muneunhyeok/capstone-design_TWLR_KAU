# -*- coding: utf-8 -*-
"""TWLR 고관절/휠 체결부 볼트 조인트 검토 — 마찰전달 기준"""
import math
def preload(T, d, k=0.20): return T/(k*d)          # N,  T[N·m], d[m]
def fric(Fp, mu=0.15, n_if=1): return mu*Fp*n_if

print("="*100)
print(" 체결부별 마찰 전달 여유  (K=0.20, μ=0.15, 단일 마찰면)")
print("="*100)
hdr=f"{'체결부':32s}{'볼트':>10s}{'PCD':>7s}{'토크':>8s}{'요구력':>9s}{'조임T':>8s}{'예압':>9s}{'마찰용량':>10s}{'안전율':>8s}"
print(hdr); print("-"*100)
cases=[
 ("허브 ↔ AK45 출력",      "6-M3", 0.003, 27, 24.0, 1.3, 6),
 ("마운트링 ↔ AK45 스테이터","6-M3", 0.003, 48, 24.0, 1.3, 6),
 ("허브 ↔ 허벅지 링크",     "8-M4", 0.004, 60, 24.0, 3.0, 8),
 ("마운트링 ↔ 토르소",      "6-M4", 0.004, 60, 24.0, 3.0, 6),
 ("Shin ↔ WA172E 휠",      "4-M6", 0.006, 36, 13.0, 10.0, 4),
]
for nm,spec,d,pcd,Tq,Tt,n in cases:
    F_req = Tq/(pcd/2000.0)
    Fp = preload(Tt,d); cap = fric(Fp)*n
    print(f"{nm:32s}{spec:>10s}{pcd:>7.0f}{Tq:>8.1f}{F_req:>9.0f}{Tt:>8.1f}{Fp:>9.0f}{cap:>10.0f}{cap/F_req:>8.2f}")

print()
print("="*100)
print(" 다웰핀 (3 x Ø4, PCD22) 전단 용량 — 허브↔AK45 출력")
print("="*100)
A=math.pi*4**2/4
for tau in (150,200):
    cap=3*A*tau; Tcap=cap*0.011
    print(f"   허용전단 {tau} MPa → 핀 3개 전단력 {cap:.0f} N → 전달토크 {Tcap:.1f} N·m")

print()
print("="*100)
print(" PLA 나사부 검토")
print("="*100)
# 토르소(PLA)에 M4 x 8 직접 탭
tau_pla=25.0   # MPa 전단
for d,L in ((4,8),):
    Ash = math.pi*d*L*0.5
    print(f"   PLA 직접탭 M{d}x{L}: 나사산 전단면적 {Ash:.0f} mm² → 파단하중 {Ash*tau_pla:.0f} N")
    print(f"     → 안전 예압은 그 1/3 = {Ash*tau_pla/3:.0f} N (조임토크 {Ash*tau_pla/3*0.2*d/1000:.2f} N·m 수준)")
    Fp_safe=Ash*tau_pla/3; cap=fric(Fp_safe)*6; F_req=24.0/0.030
    print(f"     → 6개 마찰용량 {cap:.0f} N  vs  요구 {F_req:.0f} N   안전율 {cap/F_req:.2f}  ← 부족")
print()
print("   황동 열간삽입 인서트(M4, 길이 8) 사용 시 인발강도 ≈ 1000~1200 N")
Fp=900; cap=fric(Fp)*6
print(f"     → 예압 {Fp} N 기준 6개 마찰용량 {cap:.0f} N  vs 요구 {F_req:.0f} N   안전율 {cap/F_req:.2f}  ← 확보")

print()
print("="*100)
print(" 볼트 머리 하부 PLA 압축응력 (Shin ↔ 휠 M6)")
print("="*100)
for desc,Do,Di,T in (("M6 소켓헤드 단독 (Ø10)",10,6.6,10.0),
                     ("M6 소켓헤드 단독 (Ø10)",10,6.6,4.0),
                     ("Ø18 대형와셔",18,6.6,4.0),
                     ("Ø18 대형와셔",18,6.6,6.0)):
    A=math.pi*(Do**2-Di**2)/4; Fp=preload(T,0.006); s=Fp/A
    print(f"   {desc:26s} 조임 {T:4.1f} N·m → 예압 {Fp:6.0f} N, 접촉면 {A:5.0f} mm² → 압축응력 {s:6.1f} MPa  "
          f"{'OK (PLA 허용 ~40)' if s<40 else '★ PLA 파손'}")
    if Do==18:
        cap=fric(Fp)*4; F_req=13.0/0.018
        print(f"        마찰 전달용량 4개 {cap:.0f} N vs 스톨 요구 {F_req:.0f} N  안전율 {cap/F_req:.2f}")

print()
print("="*100)
print(" 고관절 캔틸레버 / 백래시")
print("="*100)
M=11.4906; g=9.80665
x_brg=142.0; x_wheel=258.0; off=(x_wheel-x_brg)/1000.0
Fv=M*g/2
print(f"   AK45 출력베어링 X≈∓{x_brg:.0f} mm, 휠 접지 X≈∓{x_wheel:.0f} mm → 편심 {off*1000:.0f} mm")
print(f"   정하중: 휠 1개 접지력 {Fv:.1f} N → 전도모멘트 {Fv*off:.2f} N·m")
for G,label in ((1,"정지"),(2,"주행 요철"),(3,"착지 3G")):
    print(f"      {label:10s}: {Fv*off*G:6.2f} N·m")
print(f"   AK45 출력베어링 정정격 C0=2760 N, 유효 피치경 ~Ø40 → 모멘트 정격 ≈ {2760*0.040/4:.1f} N·m")
bl=12/60.0
print(f"   백래시 12 arcmin = {bl:.2f}° → 힙~휠 336 mm에서 유격 ±{336*math.tan(math.radians(bl)):.2f} mm")
