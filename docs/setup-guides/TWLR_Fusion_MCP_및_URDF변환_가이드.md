# TWLR — Fusion 360 MCP 연결 & URDF 변환 가이드

> 작성 목적: 다른 팀원도 처음부터 동일하게 재현할 수 있도록, 두 작업 세션(① Claude Desktop ↔ Fusion 360 MCP 연결, ② Fusion 모델 → URDF 변환 → RViz2/Gazebo 실행)을 단계별로 정리한 문서.
>
> 환경: Windows(작업 PC) + Fusion 360(교육용 라이선스) / Ubuntu 24.04 + ROS2 Jazzy + Gazebo Harmonic(gz sim 8.11.0)

---

## PART 1. Claude Desktop ↔ Fusion 360 MCP 연결 설정

### 1-1. 배경 및 핵심 문제

- 목표: Claude가 Fusion 360 디자인을 직접 읽고 조작할 수 있도록 MCP로 연결.
- **교육용(학생) 라이선스의 Fusion 360에는 공식 MCP 서버 옵션이 없음** (환경설정 → 일반에 API 섹션이 아예 안 뜸).
- 해결책: 서드파티 애드인 **`frankhommers/autodesk-fusion-mcp`** (GitHub) 사용.

### 1-2. 사전 준비

1. **Node.js 설치** — `npx` 명령이 필요하므로 미설치 시 먼저 설치.
2. Fusion 360 실행 상태 유지.
3. **Claude Desktop 앱** 사용 (⚠️ 웹 브라우저 버전 Claude에서는 로컬 MCP 도구가 작동하지 않음).

### 1-3. Fusion 애드인 설치

1. GitHub에서 `frankhommers/autodesk-fusion-mcp` zip 다운로드 후 압축 해제.
2. **폴더명/구조 주의 (가장 흔한 실수)**
   - 압축 해제 시 `autodesk-fusion-mcp-main` 폴더가 생기는데, 이 폴더를 **정확히 `AutodeskFusionMCP`로 이름 변경**해야 함.
   - 설치 위치: `%APPDATA%\Autodesk\Autodesk Fusion 360\API\AddIns\`
   - 이중 중첩 금지: `AddIns\AutodeskFusionMCP\AutodeskFusionMCP\파일들` 구조면 Fusion이 인식 못 함.
   - 올바른 구조: `AddIns\AutodeskFusionMCP\(실제 파일들)`
3. Fusion에서 **Shift + S** → Add-ins 탭 → `AutodeskFusionMCP` 활성화(파란색 토글 = 켜짐).

### 1-4. Claude Desktop 설정 파일 수정

- 파일 위치: `C:\Users\<사용자명>\AppData\Roaming\Claude\claude_desktop_config.json`
- VS Code 등으로 열어 최상위에 `mcpServers` 키 추가 (기존 키들은 그대로 두고 추가만 하면 됨):

```json
"mcpServers": {
  "autodesk-fusion-mcp": {
    "command": "npx",
    "args": [
      "mcp-remote",
      "http://127.0.0.1:8765/mcp"
    ]
  }
}
```

- ⚠️ `"type": "http"` + `"url"` 형식은 Claude Desktop이 **유효하지 않은 설정으로 거부**함. 반드시 위처럼 `npx mcp-remote` 방식 사용.

### 1-5. 적용 및 확인

1. Claude Desktop **완전 종료 필수**: 작업표시줄 우측 숨겨진 아이콘(^) → Claude 아이콘 우클릭 → 종료 → 재실행. (창만 닫으면 설정이 반영 안 됨)
2. 설정(톱니바퀴) → 개발자 → 로컬 MCP 서버 목록에서 `autodesk-fusion-mcp`가 **running** 상태인지 확인.
3. 포트 확인(선택): cmd에서 `netstat -ano | findstr :8765` → `LISTENING`/`ESTABLISHED` 표시되면 서버 정상.
4. Claude Desktop **새 채팅**에서 "Fusion에서 현재 열려있는 디자인 이름이 뭐야?"로 테스트.

### 1-6. 트러블슈팅 체크리스트

| 증상 | 원인 / 조치 |
|---|---|
| MCP 서버 목록에 항목이 안 뜸 | config 저장 확인 → Claude Desktop 트레이에서 완전 종료 후 재시작 |
| running인데 도구 호출 실패 | 웹 브라우저 버전에서 시도 중인지 확인 → Desktop 앱에서만 동작 |
| Fusion이 애드인을 인식 못 함 | 폴더명 `AutodeskFusionMCP` 정확히 일치 + 이중 중첩 여부 확인 |
| 연결 자체가 안 됨 | Node.js 설치 여부, Fusion 실행 여부, 포트 8765 점유 확인 |

---

## PART 2. Fusion 360 TWLR 모델 → URDF 변환 → RViz2/Gazebo 실행

### 2-1. 모델 개요

- 디자인 이름: **TWLR** (Two-Wheeled Legged Robot)
- 구조: 좌/우 다리 × (link1 허벅지, link2 4절링크 보조링크, link3 정강이) + 인휠모터, 4절 링크(4-bar linkage) 다리 구조
- torso에 전자부품: Teensy 4.1, Orange Pi, IMU, 1000mAh LiPo 배터리, PDB, ODrive V3.6 모터드라이버, FeeTech SM60CL 서보 ×2
- 목표: MuJoCo/Gazebo용 URDF 생성 (전자부품 포함 버전 A / 제외 버전 B 두 가지)

### 2-2. Fusion 쪽 조인트 세팅 (MCP `execute_python` 활용)

최종적으로 **As-Built Joint 18개** 구성:

**회전 관절 (Revolute) — 10개**

| Joint | 연결 | 축 |
|---|---|---|
| `hip_right` / `hip_left` | torso ↔ link1 | X |
| `aux_hip_right` / `aux_hip_left` | torso ↔ link2 | X (4-bar 상단) |
| `knee_right` / `knee_left` | link1 ↔ link3 | X |
| `link2_link3_right` / `link2_link3_left` | link2 ↔ link3 | X (4-bar 하단) |
| `wheel_right` / `wheel_left` | link3 ↔ 인휠모터 | Y (바퀴 회전) |

**고정 관절 (Rigid) — 8개**: 서보 ×2 + 전자부품 6종 → torso에 고정.
단, 전자부품은 rigid joint보다 **Rigid Group**(`torso_electronics`)으로 묶는 것이 fusion2urdf 인식률이 좋음 → torso + 전자부품 전체를 하나의 URDF 링크로 병합됨.

**Fusion API 팁 (조인트 자동 생성 시):**
- 조인트 기준점은 `JointGeometry.createByCylinderOrConeFace(face, ...)` 사용. `createByPoint`는 `Point3D`를 받지 않음(실제 BRep 지오메트리 필요).
- 원통면(핀 홀) 탐지는 반지름 오차(±0.15) + 축 방향 내적 조건을 함께 걸어야 안정적.
- 공유 컴포넌트의 월드 좌표는 로컬 origin에 transform translation을 더해 계산하되, **미러된 컴포넌트는 origin 계산이 불안정**하므로 face 인덱스 직접 접근이 안전.

### 2-3. fusion2urdf 규칙 및 주요 함정 ★핵심★

1. **As-Built Joint 방향**: `occurrenceOne = child`, `occurrenceTwo = parent` (직관과 반대!). 방향이 틀리면 kinematic tree가 뒤집혀 base_link가 leaf로 잡힘.
2. **공유 컴포넌트 문제**: 같은 컴포넌트를 좌/우 다리에 인스턴스로 재사용하면 fusion2urdf가 하나의 링크로 병합해버림 → `body.copyToComponent()`로 **독립 컴포넌트로 분리**해야 좌/우가 각각 링크로 나옴.
3. **Orphan link 오류**: kinematic tree에 연결 안 된 컴포넌트가 있으면 export 실패 → 모든 컴포넌트가 joint 또는 Rigid Group으로 tree에 연결됐는지 확인.
4. **4절 링크의 폐루프(closing loop)**: 조인트 이름에 `!closing_` 접두사를 붙여 명시적으로 표시.
5. **인코딩 버그**: `fusion2urdf/core/package_generator.py` 약 879행의 파일 open에 `encoding='utf-8'` 추가 필요 (한글 경로/이름에서 크래시).
6. **디버깅 방법**: debug 폴더의 `export_log.md`와 `snapshot.json` 확인. `snapshot.json`의 `joints[이름].occurrence_one_clean` / `occurrence_two_clean` 필드로 parent/child 해석 결과를 정확히 볼 수 있음.

### 2-4. Export 절차

1. fusion2urdf 애드인 실행: Utilities → Add-Ins → Scripts and Add-Ins → `fusion2urdf`
2. **버전 A (전자부품 포함)**: 현재 상태 그대로 export.
3. **버전 B (전자부품 제외)**: 브라우저에서 `Teensy_4.1:1`, `Orange pi:1`, `IMU sensor:1`, `1000mah LIPO battery:1`, `DIANOME PDB (1):1`, `Motor driver ODRIVE V3.6:1` 우클릭 → Suppress → export → 다시 Unsuppress.
4. Export 전 Fusion 저장 권장.

### 2-5. Ubuntu 측 빌드 및 실행

1. USB 등으로 export 산출물(ROS2 패키지, 예: `TWLR_description`)을 Ubuntu로 이동.
2. `~/ros2_ws/src`에 넣고 `colcon build` → `source install/setup.bash`.
3. **RViz2**: `robot_state_publisher` + `joint_state_publisher_gui` 런치로 확인.
4. **Gazebo Harmonic 주의사항**:
   - 링크 이름과 조인트 이름이 같은 문자열이면 충돌 → sed로 조인트 이름 수정 (예: `wheel_right` → `wheel_right_joint`).
   - 메시 경로 해석을 위해 환경변수 설정 필수:
     ```bash
     export GZ_SIM_RESOURCE_PATH=$HOME/ros2_ws/install/TWLR_description/share
     ```
   - `/tmp/TWLR.urdf` 작업 파일은 재부팅 시 사라짐 → xacro 변환 + sed 수정을 다시 실행해서 재생성해야 함.

### 2-6. 남은 이슈 (다음 작업자 참고)

- **왼쪽 다리 표시 오류**: `hip_left` 조인트가 임시방편으로 `link1_right`에 부모 연결된 상태 → URDF에서 부모를 torso로 수동 수정 필요.
- 일부 joint origin 값이 부정확 → URDF에서 수동 보정 필요.
- 서보(`servo_right`/`servo_left`)는 현재 rigid → 실제로는 link3 구동이므로 필요 시 revolute로 재정의.

---

---

## PART 3. Gazebo / MuJoCo에서 부품 수치·위치 수정하기

결론: 가능함. 다만 "어디서" 수정하느냐에 따라 방식이 다름 — Gazebo는 URDF를 직접 수정, MuJoCo는 MJCF(XML)를 직접 수정.

### 3-1. Gazebo (SDF/URDF 쪽)

Gazebo는 URDF(또는 변환된 SDF)를 그대로 읽어서 쓰기 때문에, URDF 파일 자체를 직접 수정하면 됨.

- **위치/자세**: 각 `<joint>`의 `origin xyz="x y z" rpy="r p y"` 값을 수정하면 링크 간 상대 위치가 바뀜. `hip_left` 부모 연결 문제나 joint origin 부정확 문제도 이렇게 손으로 고치면 됨.
- **질량/관성**: 각 `<link>`의 `<inertial>` 블록 (`mass value="..."`, `inertia ixx="..."` 등) 수정.
- **충돌/시각 형상 크기**: `<collision>`, `<visual>` 안의 geometry (box size, cylinder radius/length 등).
- 수정 후엔 다시 `colcon build` (xacro 쓰는 경우 재변환) → sed로 이름 충돌 재수정 → Gazebo 재실행 필요. `/tmp/TWLR.urdf`처럼 재부팅 시 사라지는 임시 파일이면 매번 재생성해야 함.

### 3-2. MuJoCo (MJCF 쪽)

손으로 만든 MJCF 모델(2족+2바퀴, 8 액추에이터)도 같은 방식으로 XML을 직접 편집하면 됨.

- **위치**: 각 `<body>` 태그의 `pos="x y z"`, `quat` 또는 `euler` 속성.
- **질량/관성**: `<inertial>` 태그의 `mass`, `diaginertia` (또는 `fullinertia`).
- **관절 특성**: `<joint>`의 `range`(가동범위), `damping`, `stiffness`, `armature` 등.
- **액추에이터 게인**: `<actuator>` 블록의 `gear`, `kp`, `forcerange` 등 — 밸런스 컨트롤러 튜닝할 때 특히 중요.
- 편집 후 `mujoco.viewer`로 다시 열면 바로 반영됨 (재컴파일 불필요, XML만 다시 로드).

### 3-3. 주의할 점

- **평형 피치각(4.53°)**: 질량 분포(배터리 위치 등)를 바꾸면 이 값도 달라지므로, 부품 위치를 수정했다면 `balance_test.py` 같은 스크립트로 새 평형점을 다시 찾아 컨트롤러 setpoint를 갱신해야 함.
- **URDF→MJCF 변환 경로를 쓰는 경우**: URDF에서 수정 → 다시 변환 순서를 지켜야 두 모델이 어긋나지 않음. MJCF에서만 고치면 다음 변환 때 덮어써짐.
- **파라미터화**(YAML+Jinja2 또는 PyMJCF)를 아직 안 넣었다면, 자주 바꿀 값들(질량, 위치 등)은 그 방식으로 옮겨두면 손 편집보다 훨씬 편해짐.

---

## 전체 파이프라인 요약 (한눈에)

```
[Windows]
 Node.js 설치
   → frankhommers/autodesk-fusion-mcp 애드인 설치 (폴더명 AutodeskFusionMCP)
   → claude_desktop_config.json에 npx mcp-remote 설정
   → Claude Desktop 완전 재시작 → running 확인
   → Claude로 Fusion 조인트 자동 세팅 (As-Built Joint 18개 + Rigid Group)
   → fusion2urdf export (인코딩 패치 적용, occurrenceOne=child 규칙 준수)

[Ubuntu 24.04 + ROS2 Jazzy]
 패키지 이동 → colcon build
   → RViz2 확인
   → sed로 이름 충돌 수정 + GZ_SIM_RESOURCE_PATH 설정
   → Gazebo Harmonic 실행
```
