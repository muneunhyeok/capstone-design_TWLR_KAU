"""TWLR balance - 설정값 (여기만 고치면 됨)

부호 관련 값(★)은 로봇에서 직접 확인하기 전까지는 '추정값'이다. README의 체크리스트대로 확인할 것.
"""
import math

# ---------------- 실행 / 통신 ----------------
TEENSY_PORT   = "/dev/serial/by-id/usb-Teensyduino_USB_Serial_20585600-if00"
TEENSY_BAUD   = 115200          # Teensy USB는 실제로 속도와 무관
CTRL_HZ       = 100             # 제어 주기 [Hz]. ODrive USB 호출이 느려서 100부터 시작, dt_max 보고 200 시도
CONTROL_CPU   = None            # 예: 7  (lscpu -e 에서 MAXMHZ 높은 큰 코어). None = 고정 안 함
LOG_DIR       = "~/logs"

# ---------------- 힙 서보 (Teensy가 500 Hz로 홀드) ----------------
HIP_KP        = 100.0           # N*m/rad  (2026-11-22 확정: 60 에서는 다리가 ~6도 처짐)
HIP_KD        = 2.0             # N*m*s/rad

# ---------------- 각도 기준 / 부호 ----------------
# Δφ(네 MATLAB 각도, + = 다리 펴짐) = DPHI_SIGN * (힙 상대각, h 기준) + DPHI_AT_H
DPHI_SIGN     = -1.0            # ★ 팀원 로그: 모터 + = 다리 처지는(접히는) 방향 → -1 추정. g 스윕 때 다리가 접히면 맞음
DPHI_AT_H     = 0.0             # ★ h 누른 자세의 Δφ [deg]. 바퀴축이 힙축 바로 아래 336 mm면 0

# IMU: 몸통 pitch 를 어느 축에서 읽을지 + 부호 (몸통 윗부분이 '전진(+바퀴) 방향'으로 기울 때 + 가 되게)
# v2 펌웨어는 이미 센서 X축 회전을 pitch 로 바꿔서 보내고, 각속도도 짝을 맞춰 보냄 (pitch <-> gy).
# → 앞뒤로 숙일 때 pitch 가 변하면 "pitch" 그대로. 각도/각속도 부호는 IMU_SIGN 하나로 같이 바뀜.
IMU_AXIS      = "pitch"         # ★ "pitch" 또는 "roll" (장착 방향에 따라)
IMU_SIGN      = +1.0            # ★
IMU_MOUNT_DEG = 0.0             # 몸통 판이 수평일 때 IMU가 읽는 값 (영점)

# CoM 오프셋 부호: CAD의 +y(무릎/몸체피벗 쪽)가 로봇 '전진' 방향이면 +1
COM_OFF_SIGN  = +1.0            # ★
COM_TRIM_DEG  = 0.0             # 모델 오차 보정. 실행 중 [ ] 또는 c 키로 조정

# ---------------- 바퀴 (ODrive, USB) ----------------
WHEEL_RADIUS  = 0.086           # m  (지름 172 mm)
TRACK_WIDTH   = 0.488           # m  (좌우 바퀴 중심 간격, CAD)
WHEEL_DIR     = (-1.0, -1.0)    # 2026-11-22 확인: 앞으로 굴리면 ODrive 값이 - (양쪽) -> -1 로 뒤집음
ODRV_VEL_LIMIT_TPS = 4.0        # ODrive 자체 속도 제한 [turns/s]
WATCHDOG_S    = 0.2             # ODrive 워치독: 파이썬이 멈추면 바퀴 자동 정지
WATCHDOG_FEED_EVERY = 5         # 제어 주기 몇 번마다 워치독 feed (100 Hz면 50 ms마다)
X_FROM_ENCODER_POS = False      # False: 위치 x 를 바퀴 속도 적분으로 계산 (USB 호출 2번 절약). True: pos_estimate 직접 읽기

# ---------------- 밸런스 제어기 (가속도 입력 → 속도 적분) ----------------
# a [m/s^2] = KX*(x-x_ref) + KV*(v-v_ref) + KTH*th_com + KW*w_com ,  v_cmd += a*dt
# 기본값 = LQR (CoM 높이 0.277 m, Q=diag(1,1,50,1), R=1)  → tools/lqr_gains.py 로 재계산
KTH           = 27.9
KW            = 4.65
KV            = 2.17
KX            = 1.0             # 1/s^2  (위치 유지, 0이면 기울기 오차만큼 계속 굴러감)
# ---------------- 제어 방식 선택 ----------------
CTRL_MODE     = "LQR"           # "LQR" 또는 "PID" (실행 중 p 키로 전환, 밸런싱 꺼져 있을 때만)
# 두 겹 PID:  th_ref = -(PID_KP_V*(v-v_ref) + PID_KI_V*∫(v-v_ref)dt)   (바깥: 속도 PI → 목표 기울기)
#             a      = PID_KP_TH*(th_com - th_ref) + PID_KD_TH*w_com     (안쪽: 기울기 PD → 바퀴 가속도)
# 기본값 = 위 LQR 과 똑같이 동작하는 값 (KP_TH=KTH, KD_TH=KW, KP_V=KV/KTH, KI_V=KX/KTH) → 여기서부터 튜닝
PID_KP_TH     = 27.9            # m/s^2 per rad
PID_KD_TH     = 4.65            # m/s^2 per rad/s
PID_KP_V      = 0.078           # rad per (m/s)   (0.1 m/s 오차 → 목표 기울기 0.45 deg)
PID_KI_V      = 0.036           # rad per m
PID_THREF_MAX_DEG = 8.0         # 바깥 루프가 만들 수 있는 목표 기울기 한계
PID_I_MAX     = 2.0             # 적분(∫v dt = 위치 오차) 한계 [m]  (2026-11-22 확정: 균형점 오차 자동 보정 범위 확대)

A_MAX         = 6.0             # m/s^2
V_MAX         = 1.0             # m/s
YAW_GAIN      = 1.0             # yaw 명령 스케일

# ---------------- 안전 ----------------
FALL_DEG          = 30.0        # |th_com| 이상이면 바퀴 정지
START_MAX_DEG     = 8.0         # b 누를 때 |th_com|이 이보다 크면 시작 거부
IMU_TIMEOUT_S     = 0.05
HIP_TIMEOUT_S     = 0.05

# ---------------- 중력 보상 (힙 피드포워드) ----------------
TFF_ENABLE    = False           # ★ 부호 확인 전엔 False. 거치대 위에서는 반드시 False (README 참고)
TFF_SCALE     = 1.0

DEG = math.pi / 180.0
