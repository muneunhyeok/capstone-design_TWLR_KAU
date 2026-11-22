"""P1: 제어 프로세스 (ODrive 입출력 + CoM 추정 + 밸런스). 이 프로세스는 혼자 돌게 둔다.
루프 안에서는 print/문자열 금지 → 숫자 튜플만 log_q 에 put_nowait.
"""
import os, time, queue
import config as C, model, shared
from shared import I, IDLE, BALANCE, FAULT, HF_HOLD, HF_FAULT

ERR_NONE, ERR_IMU_STALE, ERR_HIP_STALE, ERR_FALL, ERR_HIP_FAULT, ERR_ODRIVE, ERR_NOT_READY, ERR_TILT_START, ERR_ESTOP = range(9)

def _send(q, *cmd):
    try: q.put_nowait(cmd)
    except queue.Full: pass

def run(buf, hip_cmd_q, log_q, sim=False, calibrate=False):
    if C.CONTROL_CPU is not None:
        try: os.sched_setaffinity(0, {C.CONTROL_CPU})
        except Exception: pass

    plant = None
    if sim:
        import sim as S, odrive_io
        plant = S.Plant(th0_deg=2.0, off_err_deg=1.0)       # 모델 오차 1 deg 일부러 넣음
        wheels = odrive_io.SimWheels(plant)
    else:
        import odrive_io
        wheels = odrive_io.RealWheels(calibrate=calibrate)

    dt = 1.0 / C.CTRL_HZ
    mode = IDLE; v_cmd = 0.0; x_ref = 0.0; err = ERR_NONE
    t_next = time.perf_counter(); t_prev = t_next
    dt_max_win = 0.0; t_win = t_next; k = 0
    gs_axis_angle = I.PITCH if C.IMU_AXIS == "pitch" else I.ROLL
    gs_axis_rate  = I.GY    if C.IMU_AXIS == "pitch" else I.GX
    tff_on = False
    i_v = 0.0; th_ref = 0.0

    while not buf[I.QUIT]:
        t0 = time.perf_counter(); loop_dt = t0 - t_prev; t_prev = t0
        k += 1

        if sim:                                              # 시뮬: 플랜트 진행 + IMU 값을 공유메모리에 써줌
            plant.step(dt)
            pb, rate = plant.imu()
            raw = (pb + C.IMU_MOUNT_DEG*C.DEG) / C.IMU_SIGN
            shared.put(buf, T_TEENSY=time.monotonic(), IMU_OK=1, HIP_FLAGS=HF_HOLD | 3,
                       **{("PITCH" if C.IMU_AXIS == "pitch" else "ROLL"): raw,
                          ("GY" if C.IMU_AXIS == "pitch" else "GX"): rate / C.IMU_SIGN})

        s = shared.snap(buf)
        now = time.monotonic()

        # ---------- 상태 추정 ----------
        th_body = C.IMU_SIGN*s[gs_axis_angle] - C.IMU_MOUNT_DEG*C.DEG
        w_body  = C.IMU_SIGN*s[gs_axis_rate]
        hip_rel = 0.5*(s[I.HIP_L] + s[I.HIP_R])               # deg, h 기준
        hip_vel = 0.5*(s[I.HIP_VL] + s[I.HIP_VR])             # deg/s
        dphi    = C.DPHI_SIGN*hip_rel + C.DPHI_AT_H
        th_off  = C.COM_OFF_SIGN*model.theta_offset_deg(dphi)*C.DEG
        th_com  = th_body + th_off + s[I.TRIM_DEG]*C.DEG
        w_com   = w_body + C.COM_OFF_SIGN*model.theta_offset_slope(dphi)*C.DPHI_SIGN*hip_vel*C.DEG

        if s[I.REQ_ZERO_TRIM]:                               # c 키: 지금 자세를 균형점으로
            shared.put(buf, TRIM_DEG=s[I.TRIM_DEG] - th_com/C.DEG, REQ_ZERO_TRIM=0)

        # ---------- 안전 검사 ----------
        imu_stale = (now - s[I.T_TEENSY] > C.IMU_TIMEOUT_S) or not s[I.IMU_OK]
        hip_bad   = (int(s[I.HIP_FLAGS]) & HF_FAULT) or not (int(s[I.HIP_FLAGS]) & HF_HOLD)

        want = s[I.REQ_BALANCE] > 0.5 and not s[I.ESTOP]
        if mode == BALANCE:
            stop_err = ERR_NONE
            if s[I.ESTOP]:                 stop_err = ERR_ESTOP
            elif imu_stale:                stop_err = ERR_IMU_STALE
            elif abs(th_com) > C.FALL_DEG*C.DEG: stop_err = ERR_FALL
            elif hip_bad:                  stop_err = ERR_HIP_FAULT
            elif k % 20 == 0 and wheels.error(): stop_err = ERR_ODRIVE
            if stop_err or not want:
                wheels.disable(); mode = FAULT if stop_err else IDLE; err = stop_err
                v_cmd = 0.0
                shared.put(buf, REQ_BALANCE=0)
        elif want:                                            # 시작 요청
            if not wheels.ready:            err = ERR_NOT_READY
            elif imu_stale:                 err = ERR_IMU_STALE
            elif hip_bad:                   err = ERR_HIP_FAULT
            elif abs(th_com) > C.START_MAX_DEG*C.DEG: err = ERR_TILT_START
            else:
                wheels.enable(); mode = BALANCE; err = ERR_NONE
                v_cmd = 0.0; x_ref = wheels.position(); i_v = 0.0; th_ref = 0.0
                shared.put(buf, V_MEAS=0.0, X_MEAS=x_ref)
            if mode != BALANCE:
                shared.put(buf, REQ_BALANCE=0); mode = FAULT
        if s[I.ESTOP] and mode != BALANCE:
            wheels.disable()

        # ---------- 제어 ----------
        a = 0.0
        # 적분은 '실제로 흐른 시간'으로 (ODrive USB 가 느려서 주기가 밀리면 고정 dt 로는 게인이 줄어든 것처럼 됨)
        dti = min(max(loop_dt, 0.2*dt), 3.0*dt)
        if mode == BALANCE:
            v_ref = s[I.V_REF]; x_ref += v_ref*dti
            # 측정값은 직전 주기에 쓰기와 같이 읽어둔 값 (USB 호출 줄이기)
            v_meas = s[I.V_MEAS]; x_meas = s[I.X_MEAS]
            gsc = s[I.GAIN_SCALE]
            if s[I.CTRL_MODE] == shared.PID:
                v_err = v_meas - v_ref                                     # 바깥: 속도 PI → 목표 기울기
                i_v = max(-C.PID_I_MAX, min(C.PID_I_MAX, i_v + v_err*dti))
                lim = C.PID_THREF_MAX_DEG*C.DEG
                th_ref = max(-lim, min(lim, -(s[I.G_KP_V]*v_err + s[I.G_KI_V]*i_v)))
                a = gsc*(s[I.G_KP_TH]*(th_com - th_ref) + s[I.G_KD_TH]*w_com)   # 안쪽: 기울기 PD
            else:
                th_ref = 0.0
                a = gsc*(C.KX*(x_meas - x_ref) + C.KV*(v_meas - v_ref) + C.KTH*th_com + C.KW*w_com)
            a = max(-C.A_MAX, min(C.A_MAX, a))
            v_cmd = max(-C.V_MAX, min(C.V_MAX, v_cmd + a*dti))
            dv = C.YAW_GAIN*s[I.YAW_REF]*C.TRACK_WIDTH/2
            vl, vr = wheels.write_read(v_cmd - dv, v_cmd + dv)
            v_meas = 0.5*(vl + vr)
            if sim or C.X_FROM_ENCODER_POS:
                x_meas = wheels.position()
            else:
                x_meas = x_meas + v_meas*dti                      # 바퀴 속도 적분 (USB 호출 절약)
            shared.put(buf, V_MEAS=v_meas, X_MEAS=x_meas)
            if C.TFF_ENABLE and k % 5 == 0:
                tff = C.DPHI_SIGN*model.hip_torque_nm(dphi)*C.TFF_SCALE
                _send(hip_cmd_q, 'T', tff, tff); tff_on = True
        else:
            if tff_on:
                _send(hip_cmd_q, 'T', 0.0, 0.0); tff_on = False
            if sim:
                wheels.write_read(0.0, 0.0)

        # ---------- 기록 ----------
        dt_max_win = max(dt_max_win, loop_dt)
        if t0 - t_win > 1.0:
            shared.put(buf, DT_MAX=dt_max_win); dt_max_win = 0.0; t_win = t0
        shared.put(buf, DPHI=dphi, TH_COM=th_com, W_COM=w_com, V_CMD=v_cmd, MODE=mode, CTRL_ERR=err, TH_REF=th_ref)
        try:
            log_q.put_nowait((now, loop_dt, mode, th_body, w_body, dphi, th_off, th_com, w_com,
                              s[I.V_MEAS], v_cmd, a, s[I.X_MEAS], s[I.HIP_L], s[I.HIP_R],
                              s[I.HIP_TL], s[I.HIP_TR], s[I.V_REF], s[I.YAW_REF], err, s[I.CTRL_MODE], th_ref))
        except queue.Full:
            pass

        # ---------- 주기 맞추기 ----------
        t_next += dt
        sl = t_next - time.perf_counter()
        if sl > 0: time.sleep(sl)
        else:      t_next = time.perf_counter()               # 밀렸으면 따라잡지 말고 리셋

    wheels.disable()
