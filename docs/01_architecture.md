# 운영 아키텍처

## 기본 경로: Robot ↔ PLC ↔ Jetson

```text
현대 로봇 컨트롤러 1..N
        │ CC-Link / 현장 I/O
        ▼
  Mitsubishi 공정 PLC                    하드와이어 안전회로
        │ PLC-local mirror/command        E-stop/guard/safety PLC
        │ MC Protocol TCP                         │
        ▼                                         ▼
  plc_gateway_node  ── ROS 2 상태/서비스 ── RViz / Unity / XAI
```

Jetson은 PLC의 MC TCP client다. PLC가 pose/status를 CPU device 영역에 mirror하면
gateway가 polling하고, 승인된 service 요청을 PLC command 영역에 쓴다. 이는
“PLC가 Jetson으로 ROS를 push”하는 방식이 아니다.

물리 endpoint는 PLC 한 개다. `loading`, `unloading`은 로봇 IP instance가 아니라
PLC 내부에서 서로 겹치지 않는 register-map instance다. 현재 제공 자료에는 두
번째 현대 로봇의 PLC-local 블록이 없어 `loading`만 활성화한다.

```text
robot_system.launch.py                공개 진입점: sim/debug만
  └─ plc_bringup.launch.py            field/sim 구성, field read preflight
      ├─ fake_plc                     sim에서만
      ├─ plc_gateway_node             MC session 1개, instance N개
      ├─ unity_adapter_node           pose/status만 전달, command topic 차단
      └─ debug=true
          ├─ robot_state_publisher
          ├─ static base TF
          └─ RViz
```

## 읽기 경로

한 poll snapshot에서 instance별로 다음을 만든다.

```text
PLC pose/status device
  ├─ RobotMemory       raw register + link quality
  ├─ RobotPose         degree + raw 6-axis
  ├─ JointState        radian, RViz/TF
  ├─ RobotStatus       actual state + fresh/age
  └─ Unity adapter     /cmd_degs, /state, /mode_unity
```

actual run/hold/stop/speed는 PLC feedback에서만 만든다. 마지막으로 보낸 command를
actual처럼 echo하지 않는다. 통신이 끊기면 이전 bool을 “정상”으로 재사용하거나
가짜 E-stop을 만들지 않고 `fresh=false`, signal `UNKNOWN`으로 보낸다.

자료의 DT buffer 갱신은 약 55.1 ms(약 18 Hz)다. field의 20 Hz poll은 지연을
줄이기 위한 것이며 매번 새로운 로봇 sample을 보장하지 않는다.

## 쓰기 경로

공개 경로는 Unity command topic을 로봇으로 전달하지 않는다. 명령은 확인형 ROS
service만 받으며, 세 단계를 구분한다.

```text
ROS service request
  → accepted          Jetson policy/설정 통과
  → controller_ack    PLC MC write response 정상
  → register_readback command register 재조회 일치
  → confirmed         별도 PLC actual feedback/ack 재조회 일치
```

부팅, 재접속, timeout만으로는 어느 register에도 쓰지 않는다. 특히 run bit를 자동
복원하지 않는다. `100%` 요청은 D1016/D1018/D1020만 0으로 만들고 START하지 않는다.

`request_stop`은 PLC의 비안전 공정 정지 요청이다. 물리 비상정지 서비스가 아니다.
네트워크가 끊기면 Jetson은 같은 네트워크로 stop을 보낼 수 없으므로 PLC ladder가
Jetson heartbeat timeout을 감시해 자체적으로 제한 상태로 전환해야 한다.

## field와 sim

```bash
ros2 launch robot_bridge robot_system.launch.py
ros2 launch robot_bridge robot_system.launch.py sim:=false debug:=true
```

- `sim`: localhost fake PLC가 같은 No.9~17 register 계약을 제공한다. pose,
  speed/Hold/action service, RViz/Unity 데이터 경로를 시험한다.
- 기본 인자는 `sim:=true debug:=true`이다. sim의 PLC 축좌표는
  `[38.56, 136.25, -49.48, 0.17, -86.85, -50.68]` degree이며,
  RViz/Unity에는 CAD 변환 `[-S,H-90,-V,-R2,B,-R1]`을 적용해
  `[-38.56, 46.25, 49.48, -0.17, -86.85, 50.68]`로 표시한다. `Hold=true`로
  시작하고 `set_hold(false)` 서비스로 애니메이션을 시작한다.
- `field`: read-only preflight 후 gateway를 만들며 No.9~17 write는 별도의
  `allow_field_control_writes` opt-in이 있어야 열린다. controller 좌표와
  CAD 시각화 변환은 sim과 같은 계약을 사용한다.
- `debug`: 데이터 원천이나 write policy를 바꾸지 않고 TF/RViz만 추가한다.

## 현장 계약이 필요한 이유

참고자료의 `D1000...`은 현대 로봇 측 memory 표이고 `RwrD1000...`은 상위 통신
buffer 표기다. 이 값이 Main PLC CPU에서 MC로 접근할 실제 `D/W/R/ZR` 주소와
같다는 증거가 없다. `RwrD2000`, `RwrD3000`도 PLC `D2000`, `D3000`으로 단정할
수 없다. 그래서 sample map은 sim에서만 확정값처럼 쓰고 field는 아래 계약을
받기 전 fail-closed다.

- PLC CPU/Ethernet 모듈, IP/subnet, MC Open Setting
- TCP port(5000~5009 제외), 3E/4E, binary/ASCII
- instance별 pose/status/request/actual/ack device
- DWORD word order, unit/scale/sign/offset
- command level/pulse, one-hot, reset timing, reject result
- heartbeat/sequence/ack와 PLC-side communication-loss policy

## 별도 Hi6 direct 경로

`hi6_bringup.launch.py`와 `docs/07_hi6_direct.md`는 PLC를 거치지 않는 컨트롤러
Open API 실험용으로 보존한다. `robot_system.launch.py`에서는 include하지 않으며
현 공정 운영 구조의 fallback이나 자동 선택 경로가 아니다.
