# -*- coding: utf-8 -*-
import io, contextlib, math, json, re, os
os.chdir(os.path.dirname(os.path.abspath(__file__)))
src = open('mass_cog.py',encoding='utf-8').read()
hip = open('hip_torque3.py',encoding='utf-8').read()
print(f"{'벽':>6s}{'인필':>6s}{'총질량[kg]':>12s}{'CoG_Z[mm]':>11s}{'지면높이':>10s}"
      f"{'링크 6개 PLA[g]':>16s}{'τ_hip@Δθ=0':>13s}{'정격 대비':>12s}")
print('-'*92)
res=[]
for wall,infill in [(2.4,0.15),(2.4,0.20),(2.4,0.30),(2.4,0.40),(4.0,0.20),(4.0,0.40)]:
    s = re.sub(r'^INFILL   = [0-9.]+', 'INFILL   = %.2f'%infill, src, flags=re.M)
    s = re.sub(r'^WALL_T   = [0-9.]+', 'WALL_T   = %.1f'%wall, s, flags=re.M)
    s = s.replace('__file__', repr(os.path.abspath('mass_cog.py')))
    g={'__name__':'__main__'}
    with contextlib.redirect_stdout(io.StringIO()): exec(compile(s,'m','exec'),g)
    M=g['M']; CZ=g['CZ']
    rows=g['rows']
    link = sum(r['m'] for r in rows if r['mk']=='PLA' and any(k in r['name'] for k in ('Thigh','Shin','Coupler')))
    h={'__name__':'__main__'}
    with contextlib.redirect_stdout(io.StringIO()): exec(compile(hip.replace('__file__', repr(os.path.abspath('hip_torque3.py'))),'h','exec'),h)
    t0=abs(h['tau'](0.0))
    res.append((wall,infill,M/1000,CZ,CZ+336+86,link,t0))
    print(f"{wall:6.1f}{infill*100:5.0f}%{M/1000:12.3f}{CZ:11.1f}{CZ+336+86:10.1f}"
          f"{link:16.0f}{t0:13.2f}{t0/h['T_RATED']*100:11.0f}%")
print(f"(정격 = {h['ACT']['name']} {h['T_RATED']:g} N·m)")
json.dump(res,open('infill_sweep.json','w'))
