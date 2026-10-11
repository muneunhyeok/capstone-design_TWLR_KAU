# DM-J4340P-2EC 초기 설정 (모터별 1회)

출처: 2학기 통합본 발표 슬라이드 62 · 101 · 98 — 해설은 [`docs/electronics/01-can-integration-plan.md`](../docs/electronics/01-can-integration-plan.md) §5

> **모터는 한 개씩 연결해서 설정합니다.** 출하 ID 가 같으면 CAN 버스에서 충돌합니다.

## 준비물

- USB-TTL 3.3 V (또는 Damiao USB2CAN) + 모터 동봉 GH1.25 3핀 디버그 케이블
- 24 V 전원공급기 (전류제한 3 A)
- Windows PC — Damiao 디버깅 툴

## 설정값

| 레지스터 | 번호 | 왼쪽 (L) | 오른쪽 (R) | 비고 |
|---|---:|---|---|---|
| CAN_ID (ESC_ID) | 8 | `0x01` | `0x02` | 명령 프레임 ID |
| Master ID | 7 | `0x11` | `0x12` | 피드백 프레임 ID |
| Control Mode | 10 | 1 = MIT | 1 = MIT | |
| PMAX / VMAX / TMAX | 21–23 | 12.5 / 8 / 28 | 12.5 / 8 / 28 | **펌웨어 `config.h` 와 반드시 같게** — 다르면 모든 값이 비례해서 틀어짐 |
| CAN Timeout | 9 | 2000 | 2000 | 2000 × 50 µs = 100 ms 명령 없으면 모터 스스로 정지 |

## 순서

1. **연결** — 모터 24 V(전류제한 3 A), 디버그 포트 ↔ USB-TTL (TX ↔ RX 교차)
2. **읽기** — Damiao 디버깅 툴 → 포트 선택(921600) → Open Port → Read Param
3. **쓰기 · 저장** — 위 표대로 입력 → Write → 저장. 전원을 다시 넣고 한 번 더 읽어 확인
4. **영점** — 다리를 기준 자세(Δφ = 0, 3주차 정의)에 맞춘 상태에서 Set Zero
   (`dm_single_test` 의 `z` 키로도 가능)
5. **확인** — `dm_single_test` 에서 `e` → 피드백 state 1, 각도 0 근처

## 툴이 없을 때 — Teensy 에서 레지스터 쓰기 (ID `0x7FF`)

```cpp
tx(m.disable());
tx(m.writeRegU32(dm::reg::MST_ID, 0x12));  // 피드백 ID
tx(m.writeRegU32(dm::reg::ESC_ID, 0x02));  // CAN ID
tx(m.saveParams());                        // 저장 후 재부팅
```

## 특수 명령 프레임 (데이터 8 B)

| 데이터 | 동작 |
|---|---|
| `FF FF FF FF FF FF FF FC` | Enable |
| `FF FF FF FF FF FF FF FD` | Disable |
| `FF FF FF FF FF FF FF FE` | 현재 위치를 영점으로 |
| `FF FF FF FF FF FF FF FB` | 에러 클리어 (펌웨어 버전에 따라 지원) |

피드백 state 코드: `0` Disable · `1` Enable · `8` 과전압 · `9` 저전압 · `A` 과전류 · `B` MOS 과열 · `C` 코일 과열 · `D` 통신두절 · `E` 과부하 (8 이상이면 펌웨어가 FAULT 처리)
