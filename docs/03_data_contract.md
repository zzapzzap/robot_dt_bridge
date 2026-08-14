# 데이터 계약 (Interface Contract)

브리지 배치가 바뀌어도 **이 표만 지키면 Unity 는 손댈 필요가 없다.**
CDR 결과보고서 Ⅳ.5.자 「인터페이스 데이터요소 목록」의 IE-009 ~ IE-011 에 대응한다.

---

## 1. ROS 2 → Unity

| 토픽 | 타입 | 길이 | 단위 · 순서 | 주기 |
|---|---|---|---|---|
| `/robot/<id>/cmd_degs` | `std_msgs/Float64MultiArray` | 6 | **degree**, `[J1…J6]` = 로딩 `[S,H,V,R2,B,R1]` / 언로딩 `[S,L,U,R,B,T]` | 18~20 Hz |
| `/robot/<id>/state` | `std_msgs/Int32MultiArray` | 7 | `[run, hold, estop, sd1, sd2, sd3, op_state]` | 18~20 Hz |
| `/worker/unity/bodies` | `std_msgs/Float32MultiArray` | 가변 | `[n, id0, x,y,z ×28, id1, …]` · m · `stag_marker` 기준 | 20 Hz |

> **degree 로 보내는 이유** — 기존 GP8 파이프라인과 동일 계약을 유지하기 위함이다.
> Unity 내부에서 radian 변환을 하지 않는다.

## 2. Unity → ROS 2

| 토픽 | 타입 | 길이 | 순서 |
|---|---|---|---|
| `/robot/<id>/unity_command` | `std_msgs/Int32MultiArray` | 6 | `[run, hold, stop, sd1, sd2, sd3]` |

Unity 명령의 우선순위는 **0** 이다. XDI(정지 100) · XAG(감속 20~60) 지령을
덮어쓰지 못한다 — 안전상 의도된 설계다.

## 3. ROS 2 내부 (커스텀 msg)

| 토픽 | 타입 | 용도 |
|---|---|---|
| `/robot/<id>/memory` | `robot_bridge_msgs/RobotMemory` | PLC 원시값 · 통신 품질 (기록 · 진단) |
| `/robot/<id>/cmd_degs_raw` | `robot_bridge_msgs/RobotPose` | 변환 결과 + 원시값 + 교정 여부 |
| `/robot/<id>/command` | `robot_bridge_msgs/RobotCommand` | XDI · XAG · Unity 가 발행하는 제어 명령 |

## 4. 왜 std_msgs 로 한 번 더 변환하나

Unity 의 ROS-TCP-Connector 는 커스텀 msg 를 쓰려면 **C# 코드 생성**이 필요하다.
메시지를 고칠 때마다 Unity 프로젝트를 다시 만져야 해서, 실무에서 자주 깨진다.
`unity_adapter_node` 가 std_msgs 로 눌러 주면 Unity 는 **표준 타입만** 알면 된다.

```
RobotPose  ──┐                         ┌── Float64MultiArray  (cmd_degs)
RobotMemory ─┼─ unity_adapter_node ────┼── Int32MultiArray    (state)
PoseArray ───┘                         └── Float32MultiArray  (bodies)
                     ▲
                     └───────────────────  Int32MultiArray    (unity_command)
```

## 5. 좌표계

| 항목 | 값 |
|---|---|
| 기준 프레임 | `stag_marker` (바닥 STag 마커, anchor id 0) |
| ROS 관례 | 오른손 · x 앞 · y 왼쪽 · **z 위** · 단위 m |
| Unity 변환 | `(x, y, z)_ros → (-y, z, x)_unity` — `WorkerPoseReceiver.convertRosToUnity` |
| 로봇 원점 ↔ 마커 | `stag_marker → robot_base` 외부 캘리브 **[확인필요]** (AI-106) |
