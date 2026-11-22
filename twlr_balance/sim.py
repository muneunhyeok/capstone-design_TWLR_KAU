"""--sim 모드용 간단한 플랜트: 바퀴 달린 역진자 (CoM 높이 h), 바퀴는 ODrive 속도 루프처럼 1차 지연.
th = CoM 기울기 (+ = 전진 쪽으로 기움). 몸통 pitch(IMU) = th - COM_OFF_SIGN*theta_off_true
"""
import math, random
import model, config as C

class Plant:
    def __init__(self, th0_deg=3.0, h=0.277, off_err_deg=0.0, tau_v=0.03):
        self.h, self.tau = h, tau_v
        self.x = self.v = 0.0; self.th = th0_deg*C.DEG; self.w = 0.0
        self.v_cmd = 0.0
        self.held = True                  # 밸런싱 시작 전엔 사람이 잡고 있는 것으로 가정
        self.off_true = (model.theta_offset_deg(C.DPHI_AT_H) + off_err_deg) * C.DEG
    def step(self, dt, n=10):
        g = 9.81; h = dt / n
        if self.held:
            self.v = 0.0; self.w = 0.0; return
        for _ in range(n):
            a = (self.v_cmd - self.v) / self.tau
            if abs(self.th) < 1.2:
                alpha = (g*math.sin(self.th) - a*math.cos(self.th)) / self.h
            else:
                alpha = 0.0; self.w = 0.0          # 넘어짐
            self.v += a*h; self.x += self.v*h
            self.w += alpha*h; self.th += self.w*h
    def imu(self):
        pitch_body = self.th - C.COM_OFF_SIGN*self.off_true + random.gauss(0, 0.002)
        rate = self.w + random.gauss(0, 0.01)
        return pitch_body, rate
