# robot_dt_bridge

현대 로딩/언로딩 로봇(YS080 / HH050) ↔ 공정 PLC ↔ Unity 디지털 트윈을 잇는
**메신저 2개**를 하나의 프로젝트로 묶은 것.

| | 메신저 | 하는 일 |
|---|---|---|
| ① | **Unity ↔ ROS 2** | 6축 pose · 정지/감속 상태를 디지털 모델에 반영, 반대로 Unity 조작 → 명령 발행 |
| ② | **ROS 2 ↔ PLC** | MELSEC MC 프로토콜로 로봇 상태를 읽고, 정지/감속 명령을 기록 |

공장 배경 없이 **로봇 + 작업자 pose** 만 그려 씬을 가볍게 유지한다.

---

## 빠른 시작 — PLC 없이 (5분)

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash

ros2 launch robot_bridge bringup.launch.py profile:=sim
```

가상 PLC(6축 사인파) + 작업자 더미 2명 + 브리지 + 어댑터가 함께 뜬다.

```bash
ros2 topic hz   /robot/loading/cmd_degs      # ≈ 20 Hz
ros2 topic echo /robot/loading/state         # [run,hold,estop,sd1,sd2,sd3,op]
ros2 topic echo /worker/unity/bodies --once  # [n, id, x,y,z ×28, …]

# 안전 모드 — NORMAL / REDUCED_SPEED_75·50·25 / PROTECTIVE_STOP / EMERGENCY_STOP
ros2 run robot_bridge mode_cli --get         # 현재 모드
ros2 run robot_bridge mode_cli rs50          # 속도제한 50 % 로 설정
ros2 run robot_bridge mode_cli --watch       # 변화 감시
```

Unity 는 빈 씬에 `DtSceneBootstrap` 하나만 붙이고 재생하면 된다.

### ROS 2 없이 프로토콜만 확인

```bash
python3 ros2_ws/src/robot_bridge_sim/robot_bridge_sim/fake_plc.py --port 5010 &
python3 tools/plc_probe.py --profile sim
```

---

## 현장 투입

```bash
# 1) config/plc.yaml 의 field 프로파일에 실제 IP·포트 기입
# 2) 접속 진단
python3 tools/plc_probe.py --host 192.168.0.10 --port 5000
# 3) 기동
ros2 launch robot_bridge bringup.launch.py profile:=field
```

막히면 → **`docs/02_plc_setup.md` 4장 「증상별 원인표」**

---

## 구성

```
robot_dt_bridge/
├── config/
│   ├── plc.yaml          PLC 접속 · D레지스터 맵 · 안전 우선순위   ← 여기만 고치면 됨
│   ├── robots.yaml       로봇 정의 · scale/dir/offset · 토픽
│   └── network.yaml      IP · 게이트웨이 · 포트 · 점검 순서
├── ros2_ws/src/
│   ├── robot_bridge_msgs/    RobotMemory · RobotCommand · RobotPose · SafetyMode
│   │                         srv : SetSafetyMode · GetSafetyMode
│   ├── robot_bridge/
│   │   ├── mc_client.py          MC 3E/4E 바이너리 (표준 라이브러리만)
│   │   ├── config_loader.py      yaml → 설정 객체 · raw→degree 변환
│   │   ├── safety_gate.py        XDI↔XAG 명령 중재 · 안전 모드 (AI-102)
│   │   ├── mode_cli.py           모드 설정 · 조회 CLI
│   │   ├── robot_memory_node.py  ★ 메신저 ②
│   │   └── unity_adapter_node.py ★ 메신저 ①
│   └── robot_bridge_sim/     가상 PLC · 작업자 더미
├── unity/Assets/Scripts/     C# 6종 (씬 파일 없음 — 코드로 구성)
├── tools/plc_probe.py        접속 진단 CLI
└── docs/                     구조 · PLC 설정 · 데이터 계약 · 캘리브레이션 · 안전 모드
```

### 안전 모드 명칭

이름에 **결과 속도**를 박아 넣어 해석의 여지를 없앴다.

```
REDUCED_SPEED_50   =  전속의 50 % 로 운전
```

「감속 N」 표기는 감속률인지 잔여속도인지 갈려 실제로 정반대 값이 돌아다녔다.
PLC 필드 `speed_down_N` 은 사양서 계약이라 유지하고, 뒤집는 지점은 `MODE_FIELD` 표 하나뿐이다.
상세는 `docs/05_modes.md`.

```

---

## 지금 막혀 있는 것

브리지 코드는 완성됐고 시뮬레이터로 검증까지 끝났다. 남은 건 **에이시스 회신 3건**이다.

| 항목 | 내용 | CDR 번호 |
|---|---|---|
| MC 프로토콜 개방 | 포트 5000 · TCP · 바이너리 개방 여부, RUN 중 쓰기 허용 | — |
| 쓰기 실주소 | `RwrD2000/3000` 에 대응하는 PLC 실 디바이스 | C-22 |
| 축 변환 파라미터 | `scale` · `dir` · `offset` 6축 | C-17 · C-18 · C-19 |

회신 오면 `config/*.yaml` 세 줄만 고치면 된다. 코드는 안 건드린다.

---

## 참조

- `[에이시스] 로봇통신/장희-별첨자료/robotpose_python.py` — 이 프로젝트의 출발점.
  비어 있던 `RobotMemoryClient` 를 실제 MC 프로토콜 구현으로 채웠다.
- `현대로봇파일/plc_ros_bridge_spec.md` — 데이터 계약 원안
- `[에이시스] 로봇도면/YS080(HH050)-11 rev1.STEP` — 6축 모델
- `[에이시스] 로봇통신/*.pdf` — MELSEC 통신 프로토콜 · MX Component 매뉴얼
