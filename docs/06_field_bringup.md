# PLC 현장 투입 절차

각 단계가 통과하기 전에는 다음 단계, 특히 write 단계로 넘어가지 않는다.

## 0. 사무실에서 simulation 확인

```bash
cd /home/sb/colcon_ws
colcon build --packages-select robot_bridge_msgs robot_bridge_sim robot_bridge \
  --symlink-install
source install/setup.bash
ros2 launch robot_bridge robot_system.launch.py
```

인자를 생략하면 `sim:=true debug:=true`다. 시뮬레이션은
PLC 축좌표 `S=38.56, H=136.25, V=-49.48, R2=0.17, B=-86.85,
R1=-50.68` degree(raw `[3856, 13625, -4948, 17, -8685, -5068]`)와
`Hold=true`로 시작한다. sim/field 모두 controller 좌표는 원본 부호를
유지하고, RViz/Unity에는 `[-S,H-90,-V,-R2,B,-R1]`을 적용한다.
위 자세의 시각화 값은 `[-38.56, 46.25, 49.48, -0.17, -86.85, 50.68]`이다.
다음 서비스로 Hold를 해제하고 가상 자세 변화를 시작한다.

```bash
ros2 service call /robot/loading/set_hold \
  robot_bridge_msgs/srv/SetHold "{hold: false}"
```

50% → Hold 설정/해제 → 이상해제 pulse가 `accepted/controller_ack` 및
`register_readback=true`인지 확인한다. speed의 `confirmed=false`는 actual feedback
주소가 없기 때문에 정상이다. simulation 성공은 field 동작 보장이 아니다.

## 1. 현재 확인된 물리/IP

2026-08-18 read-only 시험 결과:

```text
Jetson eno1 + 임시 192.168.10.61/24
PLC              192.168.10.30/24
ping             3/3 성공, 평균 약 1.57 ms
PLC MAC          30:be:3b:60:94:ca
TCP 9000         MC 3E Binary D1000 batch-read 성공
```

즉 케이블/L2/IP subnet은 정상이다. 기존 Jetson `192.168.0.61/24`는 PLC와 다른
subnet이라 직접 통신되지 않았고 패킷이 Wi-Fi default route로 빠졌다. PLC NIC는
`192.168.10.61/24`, gateway/DNS 없음, `never-default`로 설정한다.

```bash
nmcli device status
ip route get 192.168.10.30
ping -c 3 192.168.10.30
```

`ip route get` 결과는 PLC NIC와 `src 192.168.10.61`이어야 한다.

## 2. PLC MC Open Setting 확인

PLC 담당자에게 GX Works 화면 또는 parameter export로 아래를 받는다.

- CPU/Ethernet 모듈 형명
- TCP + MC Protocol Open Setting
- 실제 local port `9000`
- 3E/4E, binary/ASCII
- online change/RUN write 정책과 remote password

```bash
python3 tools/plc_probe.py \
  --host 192.168.10.30 --port 9000 --dump D1000 21
```

## 3. 로봇별 PLC-local map 확인

현대 자료의 sample은 다음과 같지만 Main PLC 실제주소로 확정된 것이 아니다.

```text
pose             D1000, D1002..D1013
speed request    D1016=25, D1018=50, D1020=75
control request  D1100.0~5 (bit1은 read-only)
```

로딩·언로딩마다 서로 다른 PLC CPU device range를 받아야 한다. 같은 map을 두
instance에 복사하지 않는다. `config/robots.yaml`에 실제 map, word order,
scale/sign/offset을 반영하고 펜던트의 세 자세 이상과 비교한다.

## 4. 읽기 전용 field 시작

public launch는 No.9~17 write gate를 닫은 채 읽기만 수행한다.

```bash
ros2 launch robot_bridge robot_system.launch.py sim:=false debug:=true
ros2 topic hz /robot/loading/pose
ros2 topic echo /robot/loading/control_state --once
```

확인 항목:

- 실제 pose가 움직이고 축 순서/부호/단위가 펜던트와 일치
- D1016/18/20 및 D1100.0~5가 `/control_state`와 일치
- 케이블을 빼면 `fresh=false`/UNKNOWN, 이전 상태를 정상으로 표시하지 않음
- 재연결돼도 run 또는 다른 command register write가 0건

## 5. 승인된 write FAT

fenced cell, 저속, 안전담당자 입회 조건에서만 allowlisted field write를 명시적으로
연다. raw `plc_probe --write` 대신 ROS service를 사용한다.

```bash
ros2 launch robot_bridge plc_bringup.launch.py profile:=field debug:=true \
  with_unity:=true allow_field_control_writes:=true

ros2 service call /robot/loading/set_speed_percent \
  robot_bridge_msgs/srv/SetSpeedPercent "{speed_percent: 50.0}"
ros2 service call /robot/loading/set_hold \
  robot_bridge_msgs/srv/SetHold "{hold: true}"
```

`controller_ack=true`는 MC write 성공, `register_readback=true`는 같은 명령
주소 재확인일 뿐이다. 실제 로봇 동작은 펜던트와 눈으로 별도 확인한다. D1100.1
비상정지는 서비스로 쓰지 않는다.

## 6. fault injection

- Jetson node kill/restart: 자동 command write/자동 run 없음
- PLC cable pull: PLC 자체 watchdog으로 제한 상태, ROS는 stale/UNKNOWN
- PLC reboot: 재접속 후 read-only 상태부터 시작
- invalid/중복 map: launch 전 config validation 실패
- ACK timeout/reject: service `confirmed=false`, 원인을 log/status에 보존

현장 Go 조건은 네트워크가 정상일 때의 명령 성공만이 아니라, 통신·Jetson 장애
중에도 PLC와 독립 안전회로가 예상 상태를 유지하는 것이다.
