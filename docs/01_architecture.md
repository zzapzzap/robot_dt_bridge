# 구조 — 메신저 2개

```
                    ┌───────────────────────── 메신저 ① ─────────────────────────┐
                    │                                                            │
   ┌────────┐  CC-Link  ┌─────────┐  Ethernet  ┌──────────────┐   ROS 2   ┌──────────────┐
   │ Robot  │◄─────────►│ 공정 PLC │◄──────────►│ robot_bridge │◄─────────►│    Unity     │
   │ YS080  │  2.5 Mbps │ MELSEC  │  MC 3E     │  (Host AGX)  │  TCP-EP   │ Digital Twin │
   └────────┘           └─────────┘            └──────┬───────┘           └──────────────┘
                    │                                 │                            │
                    └───────────── 메신저 ② ──────────┘                            │
                                                      │                            │
                                          ┌───────────▼──────────┐                 │
                                          │  XDI (정지) · XAG    │                 │
                                          │  (감속) · MVP(작업자) │─────────────────┘
                                          └──────────────────────┘
```

---

## 메신저 ② — ROS 2 ↔ PLC  (`robot_memory_node`)

**읽기** `D1000~D1013` 20 Hz 폴링 → `RobotMemory` · `RobotPose` 발행
**쓰기** `RobotCommand` 구독 → `safety_gate` 중재 → `D2000~/D3000~` 기록 (19 Hz 재기록)

- MC 프로토콜 3E 바이너리를 **표준 라이브러리만으로** 구현 (`mc_client.py`)
- 연결 끊기면 지수 백오프 재접속, 그 사이 `link_ok=false` 를 계속 알림
- `watchdog_timeout_ms` 초과 시 `fail_safe`(기본 hold)를 강제 기록

## 메신저 ① — Unity ↔ ROS 2  (`unity_adapter_node` + C# 4종)

커스텀 msg ↔ `std_msgs` 양방향 변환. Unity 는 **코드 생성 없이** 표준 타입만 쓴다.

| Unity 스크립트 | 역할 |
|---|---|
| `DtBridgeConfig` | IP · 토픽 · 배열 인덱스를 한 곳에서 관리 |
| `RobotPoseReceiver` | `cmd_degs[6]` → 6축 관절 회전 (보간 · home 보정 · sign 뒤집기) |
| `RobotStateReceiver` | `state[7]` → 로봇 색상 · 경광등 · 상태 라벨 |
| `WorkerPoseReceiver` | `bodies` → 28관절 스켈레톤 런타임 생성 (최대 5인) |
| `SafetyCommandSender` | 조작 패널 → `unity_command[6]` 발행 |
| `DtSceneBootstrap` | 빈 씬에서 위 전부를 코드로 구성 |

---

## 왜 씬을 가볍게 했나

기존 `robot_workshell` 은 HDRP 공장 씬이라 데이터가 무겁고 로딩이 길다.
이 프로젝트는 **바닥 그리드 + 로봇 + 사람** 만 그린다.

| | robot_workshell | robot_dt_bridge |
|---|---|---|
| 배경 | HDRP 공장 전체 | 그리드 라인만 |
| 로봇 | STEP 임포트 필수 | 프리미티브 대체 모델 자동 생성 (STEP 도 꽂을 수 있음) |
| 사람 | 리깅된 캐릭터 | 구 + 원통 스켈레톤 (런타임 생성) |
| 씬 파일 | 수백 MB | **0** — 코드로 구성 |

성능이 필요해지면 `robotRoot` 에 STEP 모델을 꽂고 `autoBuildPlaceholder` 를 끄면 된다.

---

## 안전 중재 (`safety_gate.py`)

CDR 후속조치 **AI-102 「XDI ↔ XAG 중재 규칙」** 의 구현부다.

| 지령 | 우선순위 | 발행자 |
|---|---|---|
| `stop` | 100 | XDI (Edge · 긴급) |
| `hold` | 80 | XDI |
| `speed_down_3` | 60 | XAG (Host · 정밀) |
| `speed_down_2` | 40 | XAG |
| `speed_down_1` | 20 | XAG |
| `run` | 0 | Unity · 운전원 |

1. 높은 우선순위가 이긴다 · 동순위면 최신이 이긴다
2. `command_timeout_ms`(300 ms) 안에 갱신 없으면 지령 소멸 → 자동 복귀
3. 긴급(run/hold/stop)과 감속(1/2/3)은 서로 독립이되, **정지·일시정지 중에는
   감속 지령을 겹쳐 쓰지 않는다**
4. PLC 링크가 끊기면 `fail_safe` 를 강제

---

## 실행 흐름

```
profile: sim                              profile: field
──────────────                            ──────────────
fake_plc_node (127.0.0.1:5010)            실제 PLC (192.168.0.10:5000)
fake_worker_node (2명 더미)                MVP multiview_pose (실 작업자)
        │                                         │
        └──────► robot_memory_node ◄──────────────┘
                        │
                 unity_adapter_node
                        │
                     Unity
```
