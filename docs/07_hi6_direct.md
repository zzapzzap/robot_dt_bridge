# Hi6 컨트롤러 직결 경로 — 별도 실험용

운영 기본은 `Robot ↔ PLC ↔ Jetson`이다. 이 문서는 PLC를 우회해 Hi6 OpenAPI를
직접 검증해야 하는 별도 실험을 위해서만 남긴다. `robot_system.launch.py`는 이
경로를 자동 선택하거나 fallback으로 사용하지 않는다.

## 적용 조건

- 실제 controller가 Hi6이고 OpenAPI가 활성화돼 있어야 한다.
- firmware/API schema와 TCP 8888을 공급사/펜던트에서 확인해야 한다.
- Hi5a나 미지원 controller에 IP/port만 지정해도 API가 생기지는 않는다.
- 네트워크 stop/speed는 safety PLC, E-stop, guard 또는 SLS를 대체하지 않는다.

## 읽기 전용 확인

`config/hi6.yaml`은 이 실험 경로만의 설정이다. 실제 IP를 확인한 뒤 먼저 GET-only
probe를 사용한다.

```bash
python3 tools/hi6_probe.py --robot loading

ros2 launch robot_bridge hi6_bringup.launch.py \
  use_mock:=false instances:=loading \
  allow_commands:=false with_unity:=false
```

필드 쓰기 gate는 기본으로 모두 닫혀 있다. 실험을 승인받은 경우에도 low speed,
fenced cell, 물리 E-stop을 확보하고 `hi6_bringup.launch.py`의 내부 FAT option을
단계별로 연다. 이 절차는 PLC 운영 경로의 config나 service 승인을 대신하지 않는다.

## 인터페이스 개요

- REST TCP 8888: pose/status/start/stop/playback 설정 후보
- socket Stream TCP 49000: 지원 firmware에서 고주기 joint state 후보
- `/api_ver`, `/versions/sysver`: capability 확인
- `/project/robot/po_cur`: pose
- `/project/rgen`: playback/mode/status

`playback_spd_rate`는 생산 재생속도 비율이지 safety-rated 실제 축/TCP 속도가 아니다.
OpenAPI HTTP success와 robot actual 적용도 구분해야 한다.

직결 구현 파일은 `hi6_client.py`, `hi6_connection.py`, `hi6_robot_node.py`,
`hi6_bringup.launch.py`, `config/hi6.yaml`이다. 현재 공정 설계 변경으로 이 파일들은
기본 launch, PLC config, PLC service의 source of truth가 아니다.
