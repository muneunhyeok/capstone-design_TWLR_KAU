"""프로세스 사이 공유 메모리 (최신값만) + 큐.
buf: multiprocessing.Array('d') - 이름으로 접근: buf[I.PITCH]
"""
import multiprocessing as mp

FIELDS = [
    # Teensy (P2 가 씀)
    "T_TEENSY", "ROLL", "PITCH", "YAW", "GX", "GY", "GZ",
    "HIP_L", "HIP_R", "HIP_VL", "HIP_VR", "HIP_TL", "HIP_TR", "HIP_FLAGS", "IMU_OK",
    "T_LOOP_HZ", "T_IMU_US", "T_IMU_HZ",                     # Teensy 진단
    # 제어 (P1 이 씀)
    "DPHI", "TH_COM", "W_COM", "V_MEAS", "V_CMD", "X_MEAS", "MODE", "DT_MAX", "CTRL_ERR",
    # 명령 (P3 가 씀)
    "REQ_BALANCE", "V_REF", "YAW_REF", "TRIM_DEG", "GAIN_SCALE", "REQ_ZERO_TRIM",
    "ESTOP", "QUIT",
    # 제어 방식 / 실시간 PID 게인 (P3 가 바꾸고 P1 이 읽음)
    "CTRL_MODE", "G_KP_TH", "G_KD_TH", "G_KP_V", "G_KI_V", "GAIN_SEL", "TH_REF",
]
class _I: pass
I = _I()
for _k, _n in enumerate(FIELDS):
    setattr(I, _n, _k)

# MODE 값
IDLE, BALANCE, FAULT = 0.0, 1.0, 2.0
# HIP_FLAGS 비트
HF_L_EN, HF_R_EN, HF_HOLD, HF_FAULT = 1, 2, 4, 8

LQR, PID = 0.0, 1.0
PID_GAINS = ["G_KP_TH", "G_KD_TH", "G_KP_V", "G_KI_V"]      # 1~4 키로 선택

def make():
    import config as C
    buf = mp.Array('d', len(FIELDS))
    buf[I.GAIN_SCALE] = 1.0
    buf[I.CTRL_MODE] = PID if C.CTRL_MODE.upper() == "PID" else LQR
    buf[I.G_KP_TH], buf[I.G_KD_TH] = C.PID_KP_TH, C.PID_KD_TH
    buf[I.G_KP_V],  buf[I.G_KI_V]  = C.PID_KP_V,  C.PID_KI_V
    hip_cmd_q = mp.Queue(maxsize=100)    # P1/P3 -> P2 (Teensy 명령)
    log_q = mp.Queue(maxsize=20000)      # P1 -> P3 (로그 튜플, put_nowait)
    return buf, hip_cmd_q, log_q

def snap(buf):
    with buf.get_lock():
        return buf[:]

def put(buf, **kw):
    with buf.get_lock():
        for k, v in kw.items():
            buf[getattr(I, k)] = v
