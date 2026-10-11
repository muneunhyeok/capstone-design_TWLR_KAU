# ODrive v3.6 — 휠 2축(WA172E × 2) + CAN 설정. odrivetool 셸에 블록 단위로 붙여넣어 실행.
# 3주차 axis0 단일 설정(odrive_wheel_config.odrivetool.py)을 2축 + CAN 토크 제어로 확장한 것.
# 출처: 2학기 통합본 발표 슬라이드 63(스크립트) · 88(줄별 해설) · 101(1회 설정 순서)
# 해설: docs/electronics/01-can-integration-plan.md §6

# ── 보드 전체 ────────────────────────────────────────────────
odrv0.can.config.baud_rate = 1000000                # Teensy CAN2 와 동일 (1 Mbps)
odrv0.config.dc_bus_undervoltage_trip_level = 19.0  # 6S 과방전 보호 (≈3.2 V/셀)
odrv0.config.brake_resistance = 2.0                 # 동봉 2 Ω 저항
odrv0.config.enable_brake_resistor = True           # 회생 전압 흡수 → DM4340P 32 V 과전압 보호 회피

# ── 축별 (axis0 = 왼쪽 휠 node 0, axis1 = 오른쪽 휠 node 1) ──
for ax, node in ((odrv0.axis0, 0), (odrv0.axis1, 1)):
    # 모터
    ax.motor.config.pole_pairs = 15                 # 30극 → 전기각 = 15 × 기계각
    ax.motor.config.torque_constant = 1             # Kt 1 N·m/A → 토크 명령 = 전류
    ax.motor.config.calibration_current = 5
    ax.motor.config.resistance_calib_max_voltage = 8  # 5 A × 1.37 Ω ≈ 6.9 V 보다 커야 함
    ax.motor.config.current_lim = 10                # 휠 최대 10 N·m (펌웨어는 8 N·m 에서 먼저 자름)
    # 엔코더 (A/B 800 PPR × 4, 인덱스 없음 → 부팅마다 오프셋 캘리브 필요)
    ax.encoder.config.mode = ENCODER_MODE_INCREMENTAL
    ax.encoder.config.cpr = 3200
    ax.encoder.config.use_index = False
    ax.config.startup_encoder_index_search = False
    # 제어 — 토크 모드 · passthrough (밸런싱 루프는 Teensy LQR 이 닫음)
    ax.controller.config.control_mode = 1           # TORQUE_CONTROL
    ax.controller.config.input_mode = 1             # PASSTHROUGH
    ax.controller.config.vel_limit = 3              # turn/s, 토크 모드에서도 초과 시 토크 감소
    # CAN
    ax.config.can.node_id = node                    # 메시지 ID = (node << 5) | cmd
    ax.config.can.encoder_rate_ms = 2               # 위치·속도 500 Hz 자동 송신
    ax.config.can.heartbeat_rate_ms = 100           # 상태·에러 10 Hz
    ax.config.watchdog_timeout = 0.2                # CAN 명령 0.2 s 끊기면 축 정지
    # ax.config.enable_watchdog = True              # ← CAN 구동 확인 후에 켬 (슬라이드 88)

odrv0.save_configuration()
odrv0.reboot()

# ── 재부팅 후 1회: 모터 캘리브레이션 (휠을 공중에 띄운 상태) ──
#    한 축씩 — 삐 소리 후 끝날 때까지 기다리고, 에러가 없을 때만 pre_calibrated 를 켠다
import time
for ax in (odrv0.axis0, odrv0.axis1):
    ax.requested_state = 4                          # AXIS_STATE_MOTOR_CALIBRATION
    time.sleep(6)
    if ax.motor.error == 0:
        ax.motor.config.pre_calibrated = True       # 잰 R · L 을 저장 → 다음 부팅엔 생략
    else:
        print("motor calib error:", ax.motor.error)  # dump_errors(odrv0) 로 원인 확인
odrv0.save_configuration()

# 확인: dump_errors(odrv0) → 에러 없음 · axis0/1 config.can.node_id = 0 / 1
# 이후 부팅마다 필요한 엔코더 오프셋 캘리브(state 7, ≈3 s)는 Teensy 가 CAN 으로 자동 요청한다.
