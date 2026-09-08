# -*- coding: utf-8 -*-
"""TWLR 링크 3D프린팅 : 적층방향 x 인필 -> 굽힘 안전율
   단면은 Fusion 모델을 0.5mm 격자로 수치적분한 실제 단면 (무릎에서 45mm 지점)"""
import math, json

SEC = {  # Z_eff [mm^3]  (셸=벽두께 t 침식, 인필은 코어 I에 비례 기여)
 '07_Shin_L': {'Z_solid':23690,'c':35.25,'A':1988,
   'std':{(3,0.20):7538,(4,0.20):8824,(6,0.20):10039,(8,0.20):12265,(10,0.20):14233,
          (6,0.15):9186,(6,0.30):11746,(6,0.40):13452,(10,0.40):16597},
   'flat':{(3,0.20):3981,(4,0.20):4825,(6,0.20):5646,(8,0.20):7214,(10,0.20):8688,
          (6,0.15):5358,(6,0.30):6221,(6,0.40):6797,(10,0.40):9646}},
 '06_Thigh_L':{'Z_solid':25039,'c':36.25,'A':2044,
   'std':{(6,0.20):11144,(10,0.20):14938,(6,0.40):14618},
   'flat':{(6,0.20):5873,(10,0.20):9017,(6,0.40):7097}},
}

# ---- 하중 -----------------------------------------------------------------
M_ROB = 11.187          # kg  (벽 6줄 / 인필 20 % 기준)
g     = 9.80665
N1    = M_ROB*g/2       # 바퀴 1개 수직반력 [N]
ARM   = 0.1968          # 임계단면 ~ 바퀴축 거리 [m] (무릎 -45 mm)
RW    = 0.086           # 바퀴 반경 [m]
MU    = 0.8             # 노면 마찰

CASE = {}
CASE['A 정지 균형']      = N1*ARM
Ft   = MU*N1
CASE['B 급가감속 1G']    = math.hypot(N1,Ft)*ARM + Ft*RW
CASE['C 3G 착지']        = 3*N1*ARM

KT = 1.8                # 형상 + 적층결함 응력집중
SIG = {'flat':17.0, 'std':7.0}   # PLA 설계허용응력 [MPa]  층방향 / 층간방향
UTS = {'flat':50.0, 'std':28.0}

print('='*100)
print(' TWLR 링크 굽힘 검토 — 적층 방향 x 인필   (임계단면 = 무릎에서 45 mm, 실제 단면 수치적분)')
print('='*100)
print(f"  로봇 질량 {M_ROB:.3f} kg / 바퀴반력 {N1:.1f} N / 모멘트팔 {ARM*1000:.1f} mm")
for k,v in CASE.items(): print(f"    {k:14s}  M = {v:6.2f} N·m")
print(f"  응력집중 Kt = {KT}   허용응력: 눕힘(층내) {SIG['flat']} MPa · 세움(층간) {SIG['std']} MPa")
print(f"  (PLA 인장 UTS 층내 {UTS['flat']} / 층간 {UTS['std']} MPa, 안전계수 ~3 / ~4 적용)")
print()

for part in ('07_Shin_L','06_Thigh_L'):
    d=SEC[part]
    print('-'*100)
    print(f" {part}   솔리드 Z = {d['Z_solid']:,} mm³ · 단면적 {d['A']} mm² · c = {d['c']} mm")
    print('-'*100)
    print(f"{'배치':>6s}{'벽':>4s}{'인필':>6s}{'Z_eff[mm³]':>12s}{'Z/Zsolid':>10s}"
          + ''.join(f"{'SF '+k.split()[0]:>9s}" for k in CASE))
    for ori in ('flat','std'):
        for (w,inf),Z in sorted(d[ori].items()):
            lab = '눕힘' if ori=='flat' else '세움'
            line=f"{lab:>6s}{w:4d}{inf*100:5.0f}%{Z:12,d}{Z/d['Z_solid']*100:9.0f}%"
            for k,Mo in CASE.items():
                sg = KT*Mo*1000/Z
                line += f"{SIG[ori]/sg:9.2f}"
            print(line)
    print()

print('='*100)
print(' 판정 기준 : SF>=2.0 안전 / 1.5~2.0 사용가능(여유 적음) / <1.5 위험')
print('='*100)
