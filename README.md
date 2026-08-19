# robot_dt_bridge

운영 기준 구조는 **현대 로봇 ↔ 공정 PLC ↔ Jetson ↔ ROS 2/Unity**다.
Jetson은 로봇 컨트롤러나 안전 PLC를 대체하지 않고, Mitsubishi PLC의
MC Protocol TCP 서버에서 자세·상태를 읽고 승인된 일반 정지/속도 요청을 쓴다.

```text
Hyundai robot(s) ── CC-Link/PLC ladder ── Mitsubishi PLC
                                              │ MC Protocol TCP
                                              ▼
                                        Jetson PLC gateway
                                  ROS topics/services ── RViz/Unity
```

MX Component는 Windows용 편의 라이브러리라 Jetson에는 설치하지 않는다.
PLC가 GX Works의 Open Setting에서 MC Protocol을 열면 이 저장소의 Linux
socket client가 같은 D/W/R/ZR device를 직접 읽고 쓸 수 있다.

## 실행은 두 줄만 기억하면 된다

```bash
# PLC 없이: sim + RViz debug가 기본으로 실행된다
ros2 launch robot_bridge robot_system.launch.py

# 실제 PLC 읽기 + RViz (쓰기는 잠김)
ros2 launch robot_bridge robot_system.launch.py sim:=false debug:=true
```

외부 옵션은 `sim`, `debug`뿐이다. `debug:=true`는 RViz/TF만 추가하며 제어
권한을 바꾸지 않는다. 인자를 생략하면 `sim:=true debug:=true`이며,
기본 launch는 항상 ROS→Unity pose/status topic을 만든다.
현재 PC에는 `ros_tcp_endpoint`가 없어 Unity PC와의 TCP socket은 별도 설치가
필요하지만 ROS 쪽 `/robot/<id>/cmd_degs` 출력은 확인할 수 있다.

## 시뮬레이션에서 서비스 확인

다른 터미널에서 workspace를 source한 뒤 다음 명령으로 No.9~17 계약을 확인한다.
기본 PLC 축좌표는 `S=38.56, H=136.25, V=-49.48, R2=0.17,
B=-86.85, R1=-50.68` degree이다. 이는 raw
`[3856, 13625, -4948, 17, -8685, -5068]`이다. sim과
field 모두 controller 값은 그대로 내보낸다. RViz/Unity에는 공통
CAD 변환을 적용하여 `[-38.56, 46.25, 49.48, -0.17, -86.85, 50.68]`로
표시한다.
시뮬레이션은 이 자세에서 `Hold=true`로 멈춘 상태로 시작한다.

```bash
# 기본 Hold 해제: 가상 로봇 자세 변화 시작
ros2 service call /robot/loading/set_hold \
  robot_bridge_msgs/srv/SetHold "{hold: false}"

# 다시 현재 자세에 고정
ros2 service call /robot/loading/set_hold \
  robot_bridge_msgs/srv/SetHold "{hold: true}"

# 50% 속도 요청
ros2 service call /robot/loading/set_speed_percent \
  robot_bridge_msgs/srv/SetSpeedPercent "{speed_percent: 50.0}"

# 이상 해제 0.25초 pulse (1=이상해제, 2=장치원점, 3=로봇원점, 4=대기위치)
ros2 service call /robot/loading/trigger_action \
  robot_bridge_msgs/srv/TriggerRobotAction "{action: 1}"
```

시뮬레이션 시작은 `/request_start`가 아니라 `set_hold(false)`를 사용한다.
`request_start`는 Hold가 이미 해제된 상태만 허용하는 별도 운전 요청이다.

정상속도 복원은 `speed_percent: 100.0`이다. 이는 D1016/D1018/D1020만 0으로
만들며 로봇을 자동 시작하지 않는다. 응답 의미는 다음처럼 분리된다.

- `accepted`: Jetson 정책이 요청을 접수했다.
- `controller_ack`: PLC가 MC write를 오류 없이 응답했다.
- `register_readback`: Hold/action 명령 레지스터가 다시 읽혔다.
- `confirmed`: 독립된 robot actual feedback을 확인했다. 현재 No.9~17에는
  actual 주소가 없으므로 속도 응답은 정상적으로 `confirmed=false`다.

네트워크 **비상정지 서비스는 의도적으로 제공하지 않는다.** 물리 E-stop,
가드, 안전 스캐너, 안전 PLC와 로봇 안전회로가 최종 권한을 가진다. Ethernet이
끊기면 Jetson도 PLC에 정지 명령을 보낼 수 없으므로 PLC ladder의 heartbeat
watchdog이 별도로 필요하다.

## ROS 입출력

| 이름 | 의미 |
|---|---|
| `/robot/<id>/pose` | PLC에서 읽은 controller 6축 좌표(deg) |
| `/robot/<id>/joint_states` | CAD 관절 규약으로 변환한 자세(rad), RViz 입력 |
| `/robot/<id>/cmd_degs` | Unity용 CAD 관절각 6축 배열 |
| `/robot/<id>/memory` | PLC raw/status와 link 품질 |
| `/robot/<id>/control_state` | No.9~17 raw command/register readback |
| `/robot/<id>/status` | 링크/신선도; 미확정 actual 필드는 UNKNOWN |
| `/robot/<id>/get_status` | 최신 actual 상태 조회 |
| `/robot/<id>/set_speed_percent` | D1016/18/20의 25/50/75/100% 요청 |
| `/robot/<id>/set_hold` | D1100.0 Hold 설정/해제 |
| `/robot/<id>/trigger_action` | D1100.2~5 pulse action |

`D1100.1` 정지/비상정지 비트는 `/control_state`에서만 읽으며 쓰기 서비스가 없다.
축 값은 PLC signed DWORD 원본을 `raw`에 보존하고 마지막 두 자리를 소수부로
해석한다. `raw=-12354`의 기본 각도는 `-123.54°`이다. controller
좌표는 sim/field 모두 원본 부호를 유지한다. RViz/Unity에만
`visual_dir=[-1,1,-1,-1,1,-1]`, `visual_offset=[0,-90,0,0,0,0]`을 적용한다.

## 확인된 No.9~17 PLC 주소

| No. | 기능 | PLC device | 쓰기 |
|---:|---|---|---|
| 9 | 감속 25% | `D1016=25` | 허용 목록 |
| 10 | 감속 50% | `D1018=50` | 허용 목록 |
| 11 | 감속 75% | `D1020=75` | 허용 목록 |
| 12 | Hold | `D1100.0` | 허용 목록 |
| 13 | 정지/비상정지 표기 | `D1100.1` | **읽기 전용** |
| 14 | 이상 해제 | `D1100.2` | pulse |
| 15 | 장치 원점 | `D1100.3` | pulse |
| 16 | 로봇 원점 | `D1100.4` | pulse |
| 17 | 로봇 대기위치 | `D1100.5` | pulse |

`img_0574.jpeg`에서 `D1018=50`은 확인됐다. 같은 주소를 다시 읽는 것은 PLC
register 확인이지 로봇이 실제로 50%가 됐다는 ACK는 아니다. `D1100` 쓰기는 다른
비트를 보존하는 masked read-modify-write이며, D1100.1은 절대 변경하지 않는다.

자료상 PLC↔상위 컨트롤러 원천 갱신은 약 55.1 ms(약 18 Hz)다. field gateway는
20 Hz로 polling할 수 있지만 일부 sample은 같은 PLC 값일 수 있다. sim은 화면
확인용으로 30 Hz를 사용한다.

## 실제 PLC 연결 전에 확정할 값

현재 [`config/plc.yaml`](config/plc.yaml)의 legacy D2000/D3000 field 항목은
의도적으로 잠겨 있다. No.9~17도 public launch에서는 읽기만 하며, 현장 입회 시험
시에만 다음처럼 명시적으로 한 번 열 수 있다.

```bash
ros2 launch robot_bridge plc_bringup.launch.py profile:=field debug:=true \
  with_unity:=true allow_field_control_writes:=true
```

그 뒤 위 서비스 중 필요한 하나만 호출한다. launch 자체는 시작·재접속 시 어떤
레지스터도 자동으로 쓰지 않는다.

1. PLC CPU/Ethernet 모듈 형명과 PLC IP/subnet
2. GX Works Open Setting의 TCP port, 3E/4E, binary/ASCII
3. 로딩·언로딩 각각의 PLC-local pose/status/command/ACK device 주소
4. DWORD word order, 각도 scale/sign/zero offset
5. command가 level인지 pulse인지, 실제 feedback 및 ack sequence 정의
6. Jetson heartbeat와 PLC 측 통신두절 hold/stop watchdog

현장에서는 MC 3E Binary read가 `192.168.10.30:9000`에서 확인됐다.
네트워크는 다음과 같다.

| 장치 | 제안값 |
|---|---|
| PLC | `192.168.10.30/24`, TCP `9000` |
| Jetson PLC NIC | `192.168.10.61/24` |
| gateway/DNS | 격리망이면 없음, `never-default` |

PLC에 물린 로봇별 IP는 Jetson 설정 대상이 아니다. Jetson이 접속하는 endpoint는
PLC 한 개이고, `loading`/`unloading`은 서로 다른 PLC register-map instance다.
현재 자료에는 두 번째 현대 로봇의 고유 PLC 주소가 없어서 `unloading`은 비활성이다.

읽기 계약을 먼저 확인한다.

```bash
python3 tools/plc_probe.py --host 192.168.10.30 --port 9000 --dump D1000 21
```

2026-08-18 확인에서 Jetson `eno1=192.168.10.61/24`, PLC ping 약 1.6 ms,
`D1000 x21` MC read 및 ROS pose 20 Hz가 성공했다. 기존 `192.168.0.61/24`는
PLC와 다른 subnet이라 직접 연결용으로 사용할 수 없다.

No.9~17 field 쓰기는 위의 명시적 launch opt-in으로만 연다. legacy
`commissioned/commands_enabled/start_enabled`는 미확정 D2000/D3000 경로이므로 계속
false로 둔다. MC write 성공은 로봇 동작 확인이 아니며 actual feedback/ACK가 없는
계약에서는 `confirmed=true`가 될 수 없다.

## 빌드

```bash
cd /home/sb/colcon_ws
colcon build --packages-select robot_bridge_msgs robot_bridge_sim robot_bridge \
  --symlink-install
source install/setup.bash
```

세부 자료는 다음에 정리돼 있다.

- [`docs/01_architecture.md`](docs/01_architecture.md): 운영 구조와 안전 경계
- [`docs/02_plc_setup.md`](docs/02_plc_setup.md): GX Works/MC 설정
- [`docs/03_data_contract.md`](docs/03_data_contract.md): register와 ROS 계약
- [`docs/06_field_bringup.md`](docs/06_field_bringup.md): 현장 단계별 검증
- [`docs/07_hi6_direct.md`](docs/07_hi6_direct.md): PLC를 거치지 않는 별도 실험 경로

`hi6_bringup.launch.py`, `hi6_*`, `config/hi6.yaml`은 삭제하지 않았지만 운영
기본 경로에서는 사용하지 않는다. 컨트롤러 Open API를 별도로 검증할 때만 쓴다.
