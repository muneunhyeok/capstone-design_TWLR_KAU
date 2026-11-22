"""TWLR 다리 모델 - 소령 MATLAB 정역학(linkweight_static.m) + CAD(assembly.stp) 질량 추정에서 뽑은 근사식.
Δφ [deg]: + = 다리 펴짐, 0 = CAD 기준 자세 (바퀴축이 힙축 아래 336 mm)
유효 범위: -21.6 ~ +20.5 deg (전달각 조건). 범위 밖은 끝값으로 자름.
부품 무게를 실측하면 tools/ 에서 다시 계산해서 계수만 바꾸면 됨.
"""
DPHI_MIN, DPHI_MAX = -21.6, 20.5

def _clip(d):
    return min(max(d, DPHI_MIN), DPHI_MAX)

def theta_offset_deg(dphi):
    """CoM 이 바퀴축 수직선에서 기울어진 각 [deg] (몸통 수평일 때, CAD +y 쪽이 +). 오차 0.1 deg"""
    d = _clip(dphi)
    return 6.188 - 0.1781*d + 0.002189*d*d - 0.000333*d*d*d

def theta_offset_slope(dphi):
    """d(theta_offset)/d(dphi) [deg/deg]"""
    d = _clip(dphi)
    return -0.1781 + 2*0.002189*d - 3*0.000333*d*d

def hip_torque_nm(dphi):
    """서 있을 때 다리 하나가 버텨야 하는 서보 토크 [N*m], + = 다리 펴는 방향.
    W = 14.7 N(몸통 무게/2), 링크 0.45/0.43/0.08 kg 가정. 오차 0.1 N*m"""
    d = _clip(dphi)
    return 6.303 - 0.07505*d + 0.002240*d*d - 0.0000508*d*d*d
