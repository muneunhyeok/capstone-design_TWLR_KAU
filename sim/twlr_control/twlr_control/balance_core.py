"""Balance controller math, kept free of ROS so it can be tested on its own.

Sign conventions (same as the URDF, ROS REP-103):
  pitch  > 0 : body tilts nose DOWN (forward)
  wheel velocity > 0 : wheel turns so the robot rolls FORWARD (both wheels)
  wheel torque   > 0 : pushes the robot FORWARD (both wheels)
  yaw rate > 0 : robot turns LEFT
"""
import math


class BalanceCore:
    def __init__(self, params):
        self.p = dict(params)
        self.reset()

    def reset(self):
        self.vel_int = 0.0      # integral of speed error [m]
        self.yaw_int = 0.0
        self.v_filt = 0.0
        self.v_ref = 0.0        # ramped speed command [m/s]
        self.fallen = False

    def step(self, dt, pitch, pitch_rate, yaw_rate, wheel_vel_l, wheel_vel_r,
             v_cmd, yaw_cmd):
        """One control step. Angles in rad, rates in rad/s, speeds in m/s.
        Returns dict with torques [N m] for each wheel and debug values."""
        p = self.p
        off = math.radians(p['pitch_offset_deg'])

        # ---- fall detection: stop the motors when the robot is down ----
        tilt = pitch - off
        if self.fallen:
            if abs(tilt) < math.radians(p['rearm_angle_deg']):
                self.reset()                      # picked back up -> run again
            else:
                return self._out(0.0, 0.0, pitch, off, 0.0)
        if abs(tilt) > math.radians(p['fall_angle_deg']):
            self.fallen = True
            self.vel_int = self.yaw_int = 0.0
            return self._out(0.0, 0.0, pitch, off, 0.0)

        # ---- forward speed estimate ----
        # wheel encoders measure rotation relative to the leg, so the body
        # pitch rate is added back to get rotation relative to the ground
        r = p['wheel_radius']
        v = r * (0.5 * (wheel_vel_l + wheel_vel_r) + pitch_rate)
        a = min(1.0, dt / max(p['vel_filter_tau'], 1e-6))
        self.v_filt += a * (v - self.v_filt)

        # ---- speed command with acceleration limit ----
        v_cmd = max(-p['max_speed'], min(p['max_speed'], v_cmd))
        dv = p['max_accel'] * dt
        self.v_ref += max(-dv, min(dv, v_cmd - self.v_ref))

        # ---- outer loop (PI): speed error -> target pitch ----
        # to go faster forward the body must lean further forward
        e_v = self.v_ref - self.v_filt
        lim = p['vel_int_limit']
        self.vel_int = max(-lim, min(lim, self.vel_int + e_v * dt))
        lean = p['kp_vel'] * e_v + p['ki_vel'] * self.vel_int
        max_lean = math.radians(p['max_lean_deg'])
        lean = max(-max_lean, min(max_lean, lean))
        pitch_ref = off + lean

        # ---- inner loop (PD): pitch error -> total wheel torque ----
        tau = p['kp_pitch'] * (pitch - pitch_ref) + p['kd_pitch'] * pitch_rate

        # ---- yaw loop (PI): turn rate error -> left/right torque split ----
        yaw_cmd = max(-p['max_yaw_rate'], min(p['max_yaw_rate'], yaw_cmd))
        e_y = yaw_cmd - yaw_rate
        ylim = p['yaw_int_limit']
        self.yaw_int = max(-ylim, min(ylim, self.yaw_int + e_y * dt))
        tau_yaw = p['kp_yaw'] * e_y + p['ki_yaw'] * self.yaw_int

        tm = p['max_wheel_torque']
        tau_l = max(-tm, min(tm, 0.5 * tau - tau_yaw))
        tau_r = max(-tm, min(tm, 0.5 * tau + tau_yaw))
        return self._out(tau_l, tau_r, pitch, pitch_ref, self.v_filt)

    def _out(self, tl, tr, pitch, pitch_ref, v):
        return dict(tau_l=tl, tau_r=tr, pitch=pitch, pitch_ref=pitch_ref,
                    v=v, v_ref=self.v_ref, fallen=self.fallen)


DEFAULTS = dict(
    pitch_offset_deg=5.8,   # balance pitch of this leg pose (acrylic on). 8.2 without acrylic
    kp_pitch=30.0,          # [N m / rad]   total for both wheels
    kd_pitch=4.0,           # [N m s / rad]
    kp_vel=0.3,             # [rad / (m/s)]
    ki_vel=0.5,             # [rad / m]
    vel_int_limit=2.0,      # [m]
    max_lean_deg=15.0,
    kp_yaw=0.5,             # [N m / (rad/s)] per wheel
    ki_yaw=1.0,
    yaw_int_limit=0.5,
    max_wheel_torque=10.0,  # [N m] per wheel  (ODrive current_lim 10 A x Kt 1)
    wheel_radius=0.086,
    vel_filter_tau=0.02,    # [s]
    max_speed=0.4,          # [m/s]
    max_accel=0.5,          # [m/s^2]
    max_yaw_rate=1.5,       # [rad/s]
    fall_angle_deg=35.0,
    rearm_angle_deg=5.0,
)
