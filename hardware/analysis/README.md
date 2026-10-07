# 해석 스크립트

전부 순수 파이썬(표준 라이브러리)으로 돌아갑니다. 입력값은 Fusion 모델 실측치가 스크립트 안에 하드코딩되어 있습니다.

| 스크립트 | 내용 |
|---|---|
| `mass_cog.py` | 부품별 질량 → 총질량 / 무게중심. PLA는 셸+인필 모델 (`INFILL`, `WALL_T` 상수). `parts.json` 출력 |
| `hip_torque3.py` | 4절 링크 폐쇄해 + 가상일 원리로 고관절 필요토크 τ(Δθ). `mass_cog.py` 를 먼저 실행해야 함 |
| `sweep_infill.py` | 인필/벽 두께를 바꿔가며 질량·무게중심·고관절 토크 변화를 표로 |
| `print_strength.py` | 적층 방향 × 인필 → 굽힘 안전율. 단면은 Fusion에서 0.5 mm 격자로 수치적분한 실제 단면 |
| `bolt_check.py` | 볼트 마찰 전달 여유, PLA 면압, 열간삽입 인서트 비교 |

```bash
python3 mass_cog.py        # → parts.json   (고관절 기본 = DM4340P)
python3 hip_torque3.py
python3 sweep_infill.py    # parts.json 을 마지막 조합으로 덮어쓰므로 마지막에 실행
python3 print_strength.py
python3 bolt_check.py      # ※ 아직 AK45 플랜지(피크 24 N·m) 기준

# 이전안(AK45-36) 재현
TWLR_ACT=AK45 python3 mass_cog.py && TWLR_ACT=AK45 python3 hip_torque3.py
```

고관절 액추에이터 사양은 `mass_cog.py` / `hip_torque3.py` 상단 `ACTUATORS` 표에 있습니다.

| 키 | 모델 | 질량 | 정격 / 피크 |
|---|---|---|---|
| `DM4340P` (기본) | Damiao DM-J4340P-2EC 24 V, 40:1 | 375 g | 9 / 27 N·m |
| `AK45` | CubeMars AK45-36 V3.0 KV80 | 349 g | 8 / 24 N·m |

결과 스냅샷: [`results/`](results/) — 접미사 없는 파일이 DM4340P, `*_AK45.txt` 가 이전안

## 해석 가정

| 항목 | 값 |
|---|---|
| PLA 밀도 | 1.24 g/cm³ |
| 셸 모델 | `f = (V_shell + infill·(V − V_shell)) / V`, `V_shell = min(A·t_wall, V)` |
| PLA 설계허용응력 | 층내(눕힘) 17 MPa · 층간(세움) 7 MPa (UTS 50 / 28 MPa 에 안전계수 3 / 4) |
| 응력집중 Kt | 1.8 |
| 볼트 | 토크계수 K = 0.20, 마찰계수 μ = 0.15 |
| 하중 케이스 | A 정지 / B 급가감속 1G (μ=0.8 + 휠토크 반력) / C 3G 착지 |
