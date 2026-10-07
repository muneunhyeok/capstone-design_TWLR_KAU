# -*- coding: utf-8 -*-
"""TWLR 질량 / 무게중심 / 힙토크 해석
   좌표계: CAD 어셈블리 좌표 (mm).  원점 = 좌우 고관절 축의 중앙
   X = 좌우(우측 +), Y = 전후, Z = 상하(위 +).  고관절 축은 (Y=0, Z=0)

   고관절 액추에이터 선택:  TWLR_ACT=DM4340P (기본, 현재) | TWLR_ACT=AK45 (이전안)
     python3 mass_cog.py                 # DM4340P
     TWLR_ACT=AK45 python3 mass_cog.py   # AK45 기준 재현
   ※ 액추에이터 형상/중심은 CAD(TWLR_assembly_AK45)의 AK45 자리값을 그대로 씀.
     DM4340P 외형 Ø57 x 56.5 는 AK45 Ø55 x 56.5 와 거의 같아 중심 위치 변화는 무시.
"""
import math, json, os

HERE = os.path.dirname(os.path.abspath(__file__))

# ---------------------------------------------------------------- 고관절 액추에이터
ACTUATORS = {
    # mass[g], rated / peak [N·m] (출력축 기준)
    "AK45":    dict(name="CubeMars AK45-36 V3.0 KV80", mass=349.0, rated=8.0, peak=24.0),
    "DM4340P": dict(name="Damiao DM-J4340P-2EC (24V, 40:1)", mass=375.0, rated=9.0, peak=27.0),
}
ACT_KEY = os.environ.get("TWLR_ACT", "DM4340P")
ACT = ACTUATORS[ACT_KEY]

# ---------------------------------------------------------------- CAD 측정값
# (name, V[mm^3], A[mm^2], cx, cy, cz, group, matkey)
P = [
 ("Part12  토르소 메인",      1486107.1, 351082.6,    0.000,  47.894,  -1.440, "BODY","PLA"),
 ("Part14  외장/프레임",      1040770.8, 369796.8,    0.000, -81.358,  43.396, "BODY","PLA"),
 ("Part15  상부 패널",         260944.8, 120573.1,    0.000, -22.011,  46.250, "BODY","PLA"),
 ("Part16  하부 패널",         282256.8, 130189.1,    0.000, -74.033, -46.250, "BODY","PLA"),
 ("Part13  배터리(6S LiPo)",   301860.0,  30122.0,    0.000,   0.000,  -7.732, "BODY","BATT"),
 ("PCB",                         8485.9,  13206.6,   -0.313, -66.403,  50.290, "BODY","PCB"),
 ("Carte PCB",                   1630.9,   2934.3,   60.318,   0.201,  49.026, "BODY","PCB"),
 ("Driver PCB v3.5",            34498.0,  33843.3,   -3.117,-124.839, -28.367, "BODY","PCB"),
 (f"20_ACT {ACT_KEY} L",             132909.4,  15890.3, -111.854,  -0.106,   0.000, "BODY","ACT"),
 (f"20_ACT {ACT_KEY} R",             132909.4,  15890.3,  111.854,   0.106,   0.000, "BODY","ACT"),
 ("22_MNT ring L",              11738.3,   7104.9, -141.500,   0.000,   0.000, "BODY","AL"),
 ("22_MNT ring R",              11738.3,   7104.9,  141.500,   0.000,   0.000, "BODY","AL"),

 ("06_Thigh_L",                604825.2,  80837.1, -187.127,  73.551, -69.323, "THIGH","PLA"),
 ("08_Thigh_R",                604825.2,  80837.1,  187.127,  73.551, -69.323, "THIGH","PLA"),
 ("21_HUB L",                   49052.1,  15467.6, -162.780,   0.000,   0.000, "THIGH","AL"),
 ("21_HUB R",                   49052.1,  15467.6,  162.780,   0.000,   0.000, "THIGH","AL"),
 ("6905 #1",                     6337.8,   4744.4, -186.500,   0.000,   0.000, "THIGH","STEEL"),
 ("6905 #2",                     6337.8,   4744.4, -177.500,   0.000,   0.000, "THIGH","STEEL"),
 ("6905 #3",                     6337.8,   4744.4,  177.500,   0.000,   0.000, "THIGH","STEEL"),
 ("6905 #4",                     6337.8,   4744.4,  186.500,   0.000,   0.000, "THIGH","STEEL"),

 ("07_Shin_L",                 577946.2,  92850.3, -213.910, 106.721,-234.895, "SHIN","PLA"),
 ("09_Shin_R",                 577946.2,  92850.3,  213.910, 106.721,-234.895, "SHIN","PLA"),
 ("10_Wheel_WA172E_L",         947417.5, 203794.9, -257.609,   4.000,-336.000, "SHIN","WA172E"),
 ("11_Wheel_WA172E_R",         947417.5, 203794.9,  257.609,   4.000,-336.000, "SHIN","WA172E"),
 ("6904 #1",                     5391.9,   4036.3, -224.500, 174.000,-164.000, "SHIN","STEEL"),
 ("6904 #2",                     5391.9,   4036.3, -215.500, 174.000,-164.000, "SHIN","STEEL"),
 ("6904 #3",                     5391.9,   4036.3,  215.500, 174.000,-164.000, "SHIN","STEEL"),
 ("6904 #4",                     5391.9,   4036.3,  224.500, 174.000,-164.000, "SHIN","STEEL"),
 ("bush @B L",                    772.8,   1646.2, -164.951, 228.500,-128.600, "SHIN","IGL"),
 ("bush @B R",                    772.8,   1646.2,  164.951, 228.500,-128.600, "SHIN","IGL"),

 ("02_Coupler_L",              116187.9,  24301.6, -162.000, 169.270, -36.729, "COUP","PLA"),
 ("03_Coupler_R",              116187.9,  24301.6,  162.000, 169.270, -36.729, "COUP","PLA"),
 ("bush @A L",                    772.8,   1646.2, -164.951, 114.000,  49.000, "COUP","IGL"),
 ("bush @A R",                    772.8,   1646.2,  164.951, 114.000,  49.000, "COUP","IGL"),
]

# ------------------------------------------------------- 재료 / 질량 모델
RHO = {"PLA":1.24, "AL":2.70, "STEEL":7.85, "IGL":1.49, "PCB":1.90}
FIXED_MASS = {"WA172E":2200.0, "ACT":ACT["mass"], "BATT":700.0}   # g, 데이터시트/실측
INFILL   = 0.20      # 인필 20 % (3D프린팅 검토 권장값)
WALL_T   = 2.4       # 벽 6줄 x 0.4 mm

def part_mass(V, A, matkey):
    """V[mm3], A[mm2] -> (질량 g, 비고)"""
    if matkey in FIXED_MASS:
        return FIXED_MASS[matkey], "데이터시트"
    if matkey == "PLA":
        Vs = min(A*WALL_T, V)                      # 셸(벽+상하면) 체적
        f  = (Vs + INFILL*(V-Vs))/V                # 유효 충전율
        return V/1000.0*RHO["PLA"]*f, f
    return V/1000.0*RHO[matkey], 1.0

rows=[]
for (nm,V,A,cx,cy,cz,grp,mk) in P:
    m,f = part_mass(V,A,mk)
    rows.append(dict(name=nm, V=V, A=A, c=(cx,cy,cz), g=grp, mk=mk, m=m, f=f))

M = sum(r["m"] for r in rows)
CX = sum(r["m"]*r["c"][0] for r in rows)/M
CY = sum(r["m"]*r["c"][1] for r in rows)/M
CZ = sum(r["m"]*r["c"][2] for r in rows)/M

print("="*104)
print(f" TWLR 질량표 — 고관절 {ACT['name']} {ACT['mass']:.0f} g   (PLA 인필 {INFILL*100:.0f} %, 벽 {WALL_T} mm 셸 모델)")
print("="*104)
print(f"{'부품':30s}{'체적[cm3]':>11s}{'재료':>9s}{'유효충전율':>11s}{'질량[g]':>10s}   중심 (X, Y, Z) mm")
print("-"*104)
for r in sorted(rows, key=lambda r:-r["m"]):
    ff = r["f"] if isinstance(r["f"],float) else 1.0
    fs = f"{ff*100:6.1f}%" if r["mk"]=="PLA" else ("  실측" if r["mk"] in FIXED_MASS else "  100%")
    print(f"{r['name']:30s}{r['V']/1000:11.1f}{r['mk']:>9s}{fs:>11s}{r['m']:10.1f}   ({r['c'][0]:7.1f},{r['c'][1]:7.1f},{r['c'][2]:7.1f})")
print("-"*104)
print(f"{'합계':30s}{sum(r['V'] for r in rows)/1000:11.1f}{'':>9s}{'':>11s}{M:10.1f}")
print()
print(f"  총질량      M = {M/1000:.3f} kg")
print(f"  무게중심  CoG = ({CX:.1f}, {CY:.1f}, {CZ:.1f}) mm   [고관절 축 원점 기준]")
print(f"  지면(휠 반경 86) 기준 CoG 높이 = {CZ+336+86:.1f} mm")
print()
grp={}
for r in rows: grp[r["g"]]=grp.get(r["g"],0)+r["m"]
for k,v in grp.items(): print(f"   그룹 {k:6s}: {v/1000:6.3f} kg")
json.dump(rows, open(os.path.join(HERE, "parts.json"), "w"))
