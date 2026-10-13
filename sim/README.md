# sim — Gazebo Harmonic 균형 시뮬레이션

CATIA 어셈블리(`assembly_urdfchange_.stp`)에서 자동 생성한 새 URDF 와 1단계(다리 고정) 균형 제어기. 출처: `twlr_sim_v1` (2026-10-12 공유).

| 폴더 | 내용 |
|---|---|
| [`twlr_description/`](twlr_description/) | xacro URDF · 메시 · 월드 · gz↔ROS 브리지 · 런치 · 바퀴 토크 부호 시험 스크립트 |
| [`twlr_control/`](twlr_control/) | `balance_node` (속도 PI → pitch PD → 바퀴 토크, yaw PI) · 게인 yaml · `balance_sim.launch.py` |
| [`verify/`](verify/) | Gazebo 없이 URDF · 메시 · 제어기를 검증하는 스크립트와 결과 |

```bash
# 저장소 루트가 ROS 패키지라서 두 폴더를 워크스페이스로 복사해서 빌드
mkdir -p ~/twlr/src && cp -r twlr_description twlr_control ~/twlr/src/
cd ~/twlr && colcon build --symlink-install && source install/setup.bash
ros2 launch twlr_control balance_sim.launch.py
```

구성 · 값 · 검증 결과 · 열린 항목: [`docs/simulation/`](../docs/simulation/)

> 루트의 `urdf/` 는 구버전 Fusion 자동생성(63.5 kg)이고, 이 폴더의 URDF(9.62 kg)와 별개입니다. CAD 해석 모델(11.24 kg)과의 차이는 [docs/simulation/02 §3](../docs/simulation/02-verification-and-open-items.md#3-시뮬-urdf-와-cad-모델의-차이) 에 있습니다.
> 이 패키지는 **실제 Gazebo 에서 구동 확인되지 않았습니다**(검증은 계산 · 평면 모델). 첫 실행 체크: [docs/simulation/02 §6](../docs/simulation/02-verification-and-open-items.md#6-gazebo-첫-실행-체크).
