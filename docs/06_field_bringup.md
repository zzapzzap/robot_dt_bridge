# 현장 투입 절차 — 랜선 꽂고 나서 무엇을 하나

**이 문서 하나만 위에서부터 따라가면 된다.** 각 단계는 「명령 → 기대 결과 → 아니면」
구조다. 막히는 지점에서 바로 다음 행동이 정해진다.

> 준비물 : Jetson AGX Orin · 랜선 · (가능하면) 에이시스 담당자 연락처

---

## 0. 클론 · 빌드  (로봇 연결 전에 미리)

```bash
git clone https://github.com/zzapzzap/robot_dt_bridge.git
cd robot_dt_bridge/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

**기대** : 3개 패키지 빌드 성공.
**아니면** : `robot_bridge_msgs` 부터 실패하면 rosidl 문제다. `sudo apt install
ros-humble-rosidl-default-generators` 후 재시도.

### 로봇 없이 전 구간 확인

```bash
ros2 launch robot_bridge bringup.launch.py profile:=sim
# 다른 터미널
ros2 topic hz /robot/loading/cmd_degs      # ≈ 20 Hz
ros2 run robot_bridge mode_cli --get
```

**여기까지는 로봇이 없어도 반드시 통과해야 한다.** 통과 못 하면 현장에서 하지 말고
사무실에서 해결하고 가는 게 맞다.

---

## 1. 랜선 연결 · 링크 확인

```bash
ip -br link                    # 상태 확인
```

**기대** : 해당 포트가 `UP`
**아니면** : 케이블 · 스위치 포트 · 상대 장비 전원. 여기서 막히면 소프트웨어 문제가 아니다.

---

## 2. 자동 탐색  ← **여기가 핵심**

IP 도 포트도 몰라도 된다. 그냥 돌린다.

```bash
python3 tools/plc_scan.py
```

이게 순서대로 한다.

| 단계 | 하는 일 |
|---|---|
| 1 | 인터페이스 · 현재 IP |
| 2 | **수동 청취** 8초 — 아무것도 안 보내고 상대가 떠드는 걸 듣는다 |
| 3 | **ARP 테이블** — MAC 이 미쓰비시 OUI 면 표시해 준다 |
| 4 | **포트 훑기** — 자국 대역 × MELSEC 상용 포트 17종 |
| 5 | **MC 신원 확인** — 열린 포트마다 `3E/4E × 바이너리/ASCII` 4조합 시도 |

**신원 확인이 핵심이다.** 포트가 열렸다고 MC 포트인 건 아니다. 실제로 MC 요청을
보내 보고 **응답이 오면** 그 포트가 맞다. 정상응답이든 `0xC056`(디바이스 범위 초과)
같은 오류응답이든 상관없다 — **요청 형식을 알아듣고 대답했다는 게 증거**다.

찾으면 이렇게 나온다.

```
찾았습니다 — 1곳
 · 192.168.0.10:5000  frame=3E  code=binary   정상응답
   D0 = [0]

config/plc.yaml 의 field 프로파일에 그대로 넣으십시오 :
  field:
    connection:
      host: 192.168.0.10
      port: 5000
      frame: "3E"
      protocol: "binary"
```

### 자국 IP 가 없거나 대역이 다를 때

우리 쪽에 그 대역 IP 가 없으면 스캔 자체가 불가능하다. 후보 대역을 하나씩 달아 본다.

```bash
sudo ip addr add 192.168.0.100/24 dev eth0
python3 tools/plc_scan.py --subnet 192.168.0.0/24
sudo ip addr del 192.168.0.100/24 dev eth0

# 미쓰비시 공장 출하 기본값이 192.168.3.x 대역인 경우가 흔하다
sudo ip addr add 192.168.3.100/24 dev eth0
python3 tools/plc_scan.py --subnet 192.168.3.0/24
```

`192.168.0` → `192.168.1` → `192.168.3` → `192.168.10` → `10.0.0` 순으로 찍어 보면
대개 걸린다. 각 대역당 1분이면 끝난다.

### IP 는 아는데 포트만 모를 때

```bash
python3 tools/plc_scan.py --host 192.168.0.10
```

---

## 3. 탐색이 실패했을 때 — 원인은 셋뿐

| # | 원인 | 확인법 | 조치 |
|---|---|---|---|
| ① | 물리 연결 | 링크 LED · `ip -br link` | 케이블 · 포트 |
| ② | 대역 불일치 | `--subnet` 으로 후보 대역 순회 | 2번 절 참고 |
| ③ | **MC 프로토콜 미개방** | `ping` 은 되는데 어떤 포트도 안 열림 | **스캔으로 못 뚫는다** |

**③ 이면 여기서 멈춘다.** 내장 Ethernet 은 기본적으로 닫혀 있고, GX Works 에서
「오픈 설정 → MC 프로토콜」을 해줘야 열린다. 우리가 Jetson 에서 할 수 있는 일이 없다.

> 이 경우 에이시스에 요청할 것 (`docs/02_plc_setup.md` 2.2)
> - 오픈 설정 : TCP · **MC 프로토콜** · 자국 포트번호
> - 교신 데이터 코드 : 바이너리 권장
> - **RUN 중 쓰기 허용** 체크 (없으면 정지/속도제한 명령을 못 내린다)
> - 원격 패스워드 해제

`ping` 이 되는데 포트가 하나도 안 열리면 ③ 이 거의 확정이다.

---

## 4. 실제 데이터 확인

```bash
# plc.yaml 에 위에서 나온 값을 넣고
python3 tools/plc_probe.py --profile field
```

**기대** : D1000~D1013 덤프 + 6축 각도가 보인다.

```
  ✓ MC 응답 정상 (end code 0x0000) — D1000 × 14
    D1000       1 (0x0001)
    ...
  6축 원시값 → 각도 (scale 0.001 가정)
    J1   raw      45000   →    45.000°
```

**아니면**

| 오류 | 뜻 | 조치 |
|---|---|---|
| `0xC056` | 디바이스 범위 초과 | D1000 대역이 안 쓰이는 것. 에이시스에 실주소 확인 (C-22) |
| `0xC05B` | 디바이스 접근 불가 | `RwrD` 가 D 가 아닐 수 있다 → `W` 로 바꿔 시도 |
| 값이 전부 0 | 통신은 되는데 데이터가 안 실림 | 로봇 운전 중인지, PLC 프로그램이 그 D 에 쓰는지 |

### `RwrD` 가 D 가 아닐 가능성 — 실제로 꽤 높다

`RwrD2000` 은 CC-Link 링크 레지스터 표기다. PLC 실 디바이스가 `D` 가 아니라
`W`(링크 레지스터)일 수 있다. 코드는 `W`·`R`·`ZR` 다 지원하니 **주소만 바꾸면 된다.**

```bash
python3 tools/plc_probe.py --profile field --dump W1000 14
python3 tools/plc_probe.py --profile field --dump R1000 14
```

---

## 5. 쓰기 확인  (조심 — 실제 로봇이 반응한다)

**로봇 주변에 사람이 없는지 확인하고, 티치펜던트 비상정지를 손에 쥔 상태에서.**

```bash
python3 tools/plc_probe.py --profile field --write D2000 0 1 0   # 보호정지
python3 tools/plc_probe.py --profile field --dump  D2000 3       # 되읽기
```

**기대** : 되읽기 값이 `[0, 1, 0]`
**아니면** : `0xC05B` 또는 되읽어도 `[0,0,0]` → **RUN 중 쓰기 금지** 상태다.
읽기만 하고 명령은 에이시스 컨트롤러 경유로 가야 한다.

---

## 6. 브리지 기동

```bash
ros2 launch robot_bridge bringup.launch.py profile:=field
ros2 topic hz /robot/loading/cmd_degs        # ≥ 18 Hz
ros2 run robot_bridge mode_cli --watch       # 모드 변화 감시
```

---

## 7. 축 캘리브레이션

지금까지는 `scale 0.001` 가정이라 각도가 실제와 다를 수 있다.

```bash
python3 tools/plc_probe.py --profile field --watch
```

띄워 놓고 티치펜던트로 **J1 만 정확히 +90°** 조그 → raw 변화량 `Δraw` 기록.

```
scale = 90 / Δraw
```

6축 다 하고 `config/robots.yaml` 에 넣은 뒤 `calibrated: true`.
상세는 `docs/04_calibration.md`.

---

## 오늘 밤 미리 해둘 것

- [ ] Jetson 에 클론 · `colcon build` 통과
- [ ] `profile:=sim` 으로 전 구간 동작 확인
- [ ] `python3 tools/plc_scan.py --host 127.0.0.1 --ports 5010` 으로 스캐너 동작 확인
- [ ] 에이시스에 **미리** 물어볼 것 정리 : MC 개방 여부 · 포트번호 · `RwrD` 실주소 ·
      RUN 중 쓰기 허용 여부 (`확인필요_기입표.md` C-17~C-24)

현장에서 제일 오래 걸리는 건 코드가 아니라 **③ 미개방** 확인 왕복이다.
가능하면 가기 전에 답을 받아 두는 게 하루를 아낀다.
