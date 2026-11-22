"""바퀴 입출력. Real = odrive 0.5.1 (USB), Sim = 시뮬레이터 바퀴.
속도 단위: 인터페이스는 m/s (바퀴 접지점 속도), 내부에서 turns/s 로 변환.
"""
import math, time
import config as C

TPS_PER_MPS = 1.0 / (2 * math.pi * C.WHEEL_RADIUS)

class RealWheels:
    IDLE, CLOSED_LOOP = 1, 8
    def __init__(self, calibrate=False):
        import odrive
        self.odrv = odrive.find_any(timeout=10)
        self.ax = (self.odrv.axis0, self.odrv.axis1)
        if calibrate:
            for a in self.ax:                     # 한 바퀴씩 (동시에 하면 전류 2배)
                a.requested_state = 3
                time.sleep(0.5)
                while a.current_state != self.IDLE:
                    time.sleep(0.1)
        for a in self.ax:
            a.controller.config.control_mode = 2  # VELOCITY
            a.controller.config.input_mode = 1    # PASSTHROUGH
            a.controller.config.vel_limit = C.ODRV_VEL_LIMIT_TPS
            a.controller.input_vel = 0
        # 워치독: 펌웨어에 속성이 '진짜' 있을 때만 켬 (없는 속성에 대입하면 에러 없이 무시되고, 나중에 feed 에서 죽을 수 있음)
        self._wd = False
        try:
            if all(hasattr(a, "watchdog_feed") and hasattr(a.config, "enable_watchdog") for a in self.ax):
                for a in self.ax:
                    a.config.watchdog_timeout = C.WATCHDOG_S
                    a.config.enable_watchdog = True
                self._wd = True
        except Exception:
            self._wd = False
        self._k = 0
        self.ready = all(a.encoder.is_ready and a.motor.is_calibrated for a in self.ax)

    def enable(self):
        for a in self.ax:
            a.error = 0; a.motor.error = 0; a.encoder.error = 0; a.controller.error = 0
            a.controller.input_vel = 0
            if self._wd: a.watchdog_feed()
            a.requested_state = self.CLOSED_LOOP

    def disable(self):
        for a in self.ax:
            try:
                a.controller.input_vel = 0; a.requested_state = self.IDLE
            except Exception:
                pass

    def write_read(self, v_l, v_r):
        """속도 명령 [m/s] 보내고 측정 속도/위치 [m/s, m] 반환"""
        a0, a1 = self.ax
        a0.controller.input_vel = C.WHEEL_DIR[0] * v_l * TPS_PER_MPS
        a1.controller.input_vel = C.WHEEL_DIR[1] * v_r * TPS_PER_MPS
        self._k += 1
        if self._wd and self._k % C.WATCHDOG_FEED_EVERY == 0:   # USB 호출 1번 ~1 ms 라 매번 안 함 (timeout 0.2 s 에 비해 충분히 자주)
            a0.watchdog_feed(); a1.watchdog_feed()
        vl = C.WHEEL_DIR[0] * a0.encoder.vel_estimate / TPS_PER_MPS
        vr = C.WHEEL_DIR[1] * a1.encoder.vel_estimate / TPS_PER_MPS
        return vl, vr

    def position(self):
        return 0.5 * (C.WHEEL_DIR[0]*self.ax[0].encoder.pos_estimate +
                      C.WHEEL_DIR[1]*self.ax[1].encoder.pos_estimate) / TPS_PER_MPS

    def error(self):
        return int(self.ax[0].error) | int(self.ax[1].error)

class SimWheels:
    """sim.Plant 와 연결된 가짜 바퀴"""
    def __init__(self, plant):
        self.p = plant; self.ready = True; self.on = False
    def enable(self):  self.on = True; self.p.held = False
    def disable(self): self.on = False; self.p.v_cmd = 0.0   # 넘어졌으면 그대로 둠
    def write_read(self, v_l, v_r):
        self.p.v_cmd = 0.5*(v_l + v_r) if self.on else 0.0
        return self.p.v, self.p.v
    def position(self): return self.p.x
    def error(self): return 0
