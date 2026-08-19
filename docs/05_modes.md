# PLC 상태와 제어 서비스

## 운영 원칙

- `/control_state`는 PLC command/register readback이다.
- 아래쪽 service request는 Jetson이 요청한 값이다.
- MC write ACK는 robot actual confirmation이 아니다.
- Unity command topic과 legacy `mode_cli`는 public PLC 쓰기 경로가 아니다.
- Ethernet command는 safety-rated E-stop/SLS가 아니다.

## 서비스

| service | PLC request | 확인 가능한 것 |
|---|---|---|
| `set_speed_percent(25)` | `D1016=25`, 나머지 감속 WORD=0 | register echo |
| `set_speed_percent(50)` | `D1018=50`, 나머지 감속 WORD=0 | register echo |
| `set_speed_percent(75)` | `D1020=75`, 나머지 감속 WORD=0 | register echo |
| `set_speed_percent(100)` | 세 감속 WORD 모두 0 | register echo |
| `set_hold(true/false)` | `D1100.0` masked set/clear | `register_readback` |
| `trigger_action(1..4)` | `D1100.2~5` 0.25초 pulse | assert/clear readback |

`100%`는 세 감속 WORD 해제만 수행하고 자동 START하지 않는다. `D1100.1`은
정지/비상정지 표기 상태로만 읽으며 쓰기 서비스가 없다.

```bash
ros2 service call /robot/loading/set_speed_percent \
  robot_bridge_msgs/srv/SetSpeedPercent "{speed_percent: 50.0}"

ros2 service call /robot/loading/set_hold \
  robot_bridge_msgs/srv/SetHold "{hold: true}"

ros2 service call /robot/loading/trigger_action \
  robot_bridge_msgs/srv/TriggerRobotAction "{action: 1}"
```

## 응답 해석

| 필드 | 의미 |
|---|---|
| `accepted` | config, permission, 입력값을 통과함 |
| `controller_ack` | PLC가 MC write를 정상 응답함 |
| `confirmed` | 후속 actual feedback/ack가 목표와 일치함 |
| `register_readback` | 명령 주소를 다시 읽어 요청값/clear를 확인함 |

public field launch에서는 `FIELD_WRITE_OPT_IN_REQUIRED`로 fail-closed한다.
command register echo만으로 `confirmed=true`를 만들지 않는다.

## 정지와 비상정지

`request_stop`은 PLC의 일반 공정 정지 request다. 기존 코드가 D2002를
`MODE_EMERGENCY_STOP`이라고 부르던 것은 안전 의미를 과장하므로 public gateway에서
사용하지 않는다. 실제 비상정지는 물리 버튼, 안전 PLC, robot safety controller가
담당한다. PLC의 D1100.1 bit는 읽기 전용 command/status register로만 표시한다.

링크가 끊긴 뒤 Jetson이 같은 Ethernet으로 hold/stop을 보낼 수는 없다. PLC ladder가
Jetson heartbeat timeout을 감시하고 자체적으로 안전한 제한 상태로 전환해야 한다.

## 호환 코드

`safety_gate.py`, `robot_memory_node.py`, `mode_cli`는 이전 프로토타입 호환을 위해
남아 있지만 `robot_system.launch.py`에서 실행하지 않는다. 특히 legacy node는
주기 command rewrite 설계이므로 field 운영에 직접 사용하지 않는다.
