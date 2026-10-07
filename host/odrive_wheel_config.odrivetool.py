# ODrive 휠모터(WA172E) 초기 설정 — odrivetool 셸에 붙여넣어 실행 (3주차 발표 슬라이드 19)
# LattePanda 에서 휠 구동 테스트에 사용. 게인은 시작값이며 이후 튜닝.

# 모터
odrv0.axis0.motor.config.pole_pairs = 15
odrv0.axis0.motor.config.torque_constant = 1
odrv0.axis0.motor.config.calibration_current = 5
odrv0.axis0.motor.config.resistance_calib_max_voltage = 8
odrv0.axis0.motor.config.current_lim = 10

# 엔코더 (J3: A, B)
odrv0.axis0.encoder.config.mode = ENCODER_MODE_INCREMENTAL
odrv0.axis0.encoder.config.cpr = 3200
odrv0.axis0.encoder.config.use_index = False
odrv0.axis0.config.startup_encoder_index_search = False

# 제어 (시작값, 이후 튜닝)
odrv0.axis0.controller.config.vel_limit = 3
odrv0.axis0.controller.config.pos_gain = 20
odrv0.axis0.controller.config.vel_gain = 0.5
odrv0.axis0.controller.config.vel_integrator_gain = 1

odrv0.save_configuration()
odrv0.reboot()
