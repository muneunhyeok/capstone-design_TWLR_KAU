"""밸런스 게인(LQR) 재계산 - 노트북에서 실행 (numpy, scipy 필요)
모델: 바퀴 가속도 a 를 입력으로 하는 역진자.  x'=v, v'=a, th'=w, w'=(g/h)th - a/h
출력 K → config.py 의 KX, KV, KTH, KW 에 그대로 넣기 (a = KX*x + KV*v + KTH*th + KW*w)
  python3 tools/lqr_gains.py --h 0.277 --q 1 1 50 1
"""
import argparse, numpy as np
from scipy.linalg import solve_continuous_are
ap = argparse.ArgumentParser()
ap.add_argument("--h", type=float, default=0.277, help="바퀴축~CoM 높이 [m] (model: dphi=0 → 0.277)")
ap.add_argument("--q", type=float, nargs=4, default=[1, 1, 50, 1], help="Q 대각: x v th w")
ap.add_argument("--r", type=float, default=1.0)
a = ap.parse_args(); g = 9.81; h = a.h
A = np.array([[0,1,0,0],[0,0,0,0],[0,0,0,1],[0,0,g/h,0]]); B = np.array([[0],[1],[0],[-1/h]])
P = solve_continuous_are(A, B, np.diag(a.q), np.array([[a.r]])); K = -(B.T @ P)/a.r
print("KX=%.3f KV=%.3f KTH=%.3f KW=%.3f" % tuple(-K.ravel()*-1))
print("closed-loop poles:", np.linalg.eigvals(A + B @ K).round(2))
