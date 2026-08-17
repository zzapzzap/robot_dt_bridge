# 안전 모드 — 서비스 · 토픽

정상 운전 / 속도제한 3단 / 보호정지 / 비상정지를 **하나의 모드 값**으로 다룬다.
설정은 서비스, 방송은 토픽이다.

---

## 1. 모드 정의

| 값 | 식별자 | 라벨 | 결과 속도 | PLC 필드 | PLC 기록 |
|---|---|---|---|---|---|
| 1 | `NORMAL` | 정상 운전 · 전속 | **100 %** | `run` | `D2000=[1,0,0]` |
| 2 | `REDUCED_SPEED_75` | 속도제한 75 % | **75 %** | `speed_down_1` | `D3000=[1,0,0]` |
| 3 | `REDUCED_SPEED_50` | 속도제한 50 % | **50 %** | `speed_down_2` | `D3000=[0,1,0]` |
| 4 | `REDUCED_SPEED_25` | 속도제한 25 % | **25 %** | `speed_down_3` | `D3000=[0,0,1]` |
| 5 | `PROTECTIVE_STOP` | 보호정지 (전원 유지) | 0 % | `hold` | `D2000=[0,1,0]` |
| 6 | `EMERGENCY_STOP` | 비상정지 | 0 % | `stop` | `D2000=[0,0,1]` |

### 왜 「감속 N」을 버렸나

`감속 2 (50 %)` 는 **50 % 를 줄인다**는 뜻인지 **50 % 로 달린다**는 뜻인지 갈린다.
실제로 이 저장소 안에서 한쪽은 25 %, 다른 쪽은 75 % 로 적혀 정반대 값이 돌아다녔다.

그래서 **이름에 결과 속도를 박아 넣었다.**

```
REDUCED_SPEED_50   =   전속의 50 % 로 운전
```

이름의 숫자와 `speed_ratio` 필드가 **항상 같다.** 어긋날 수가 없는 구조다.
(회귀 시험에서 `REDUCED_SPEED_(\d+)` 와 `speed_ratio` 일치를 자동 검사한다.)

PLC 필드명 `speed_down_N` 은 에이시스 사양서 계약이라 그대로 두고,
**숫자를 뒤집는 지점은 `MODE_FIELD` 표 하나뿐**이다.

```
speed_down_1 (25 % 감속)  →  REDUCED_SPEED_75  (속도 75 %)
speed_down_3 (75 % 감속)  →  REDUCED_SPEED_25  (속도 25 %)
```

### 정지 용어

ISO 10218 / ISO-TS 15066 을 따른다.

| | 뜻 |
|---|---|
| `PROTECTIVE_STOP` | 전원을 유지한 채 멈춤. 원인이 해소되면 복귀 가능 |
| `EMERGENCY_STOP` | 비상정지 |

## 2. 설정 — 서비스

```bash
# CLI
ros2 run robot_bridge mode_cli reduced50 --reason "작업자 접근"
ros2 run robot_bridge mode_cli estop
ros2 run robot_bridge mode_cli rs25 --hold 10        # 10초만 유지
ros2 run robot_bridge mode_cli normal --clear        # 정지 고정까지 해제

# 원형
ros2 service call /robot/loading/set_mode robot_bridge_msgs/srv/SetSafetyMode \
  "{mode: 3, source: 'operator', reason: '작업자 접근', hold_seconds: 0.0}"
```

CLI 는 `normal` `rs50` `reduced25` `estop` `pstop` · 한글 `전속` `속도50` `비상정지` ·
모드 번호 `3` · 구 표기 `감속2` 를 모두 받는다.

| 필드 | 뜻 |
|---|---|
| `hold_seconds` | `0` = 다음 설정 전까지 무기한 · `>0` = 그 시간 뒤 자동 해제 |
| `clear_latched` | 기존 고정 지령을 먼저 풀고 적용. **정지 고정을 푸는 유일한 방법** |
| 응답 `accepted` | 우선순위에 밀리면 `false` 와 사유 반환 |

## 3. 조회 · 감시

```bash
ros2 run robot_bridge mode_cli --get        # 1회 조회
ros2 run robot_bridge mode_cli --watch      # 변화 시 한 줄씩
ros2 topic echo /robot/loading/mode         # 원본 메시지
```

## 4. 토픽 vs 서비스 — 언제 뭘 쓰나

| | 토픽 `RobotCommand` | 서비스 `SetSafetyMode` |
|---|---|---|
| 쓰는 쪽 | XDI · XAG — 계속 판단해서 쏘는 자동 로직 | 운전원 · 상위 시스템 · 시험 |
| 유지 | **300 ms 후 소멸** (주기 발행 전제) | **고정(latch)** — 풀 때까지 유지 |
| 의도 | "지금 이 순간의 판단" | "지금부터 이 모드로 두어라" |

자동 로직이 주기 발행을 멈추면 지령이 저절로 풀리는 게 안전하다. 반대로 운전원이
"속도제한 50 % 로 두고 작업" 하려면 고정이 필요하다. 그래서 경로를 둘로 나눴다.

## 5. 중재 규칙

```
우선순위   EMERGENCY_STOP 100 > PROTECTIVE_STOP 80
           > REDUCED_25 60 > REDUCED_50 40 > REDUCED_75 20 > NORMAL 0
```

1. 높은 우선순위가 이긴다. 동순위면 **고정 > 토픽**, 그다음 최신
2. 서비스로 `NORMAL` 을 요청하면 속도제한 고정도 함께 풀린다
3. 단 **정지 고정은 `--clear` 없이는 안 풀린다** (안전상 의도)
4. PLC 링크 두절 시 `fail_safe`(기본 `PROTECTIVE_STOP`)가 **모든 지령보다 우선**

### 실제 동작 (검증됨)

```
서비스 REDUCED_SPEED_50 고정  → REDUCED_SPEED_50  50 %  [operator] 고정 무기한
0.4초 후                      → 유지                          ← 토픽이면 소멸했을 시점
XDI 정지 토픽 끼어듦           → EMERGENCY_STOP    0 %  [xdi]
XDI 소멸(0.3초)               → REDUCED_SPEED_50 로 복귀      ← 밑에 깔린 고정이 살아남
NORMAL 요청                   → NORMAL          100 %        ← 속도제한 고정 해제됨
정지 고정 후 NORMAL 요청       → 거절 "우선순위가 낮아…"
정지 고정 후 --clear          → NORMAL                       ← 명시적 해제
링크 두절                     → PROTECTIVE_STOP [watchdog]
```

## 6. Unity 표시

`/robot/<id>/mode_unity` (`Int32MultiArray[4]` = `[모드, 속도%, 링크정상, 고정여부]`)를
`RobotStateReceiver` 가 구독해 라벨에 표시한다.

```
속도제한 50 %   (op 1)
지령 속도제한 50 % · 속도 50 % [고정]
```

**위 줄은 PLC 가 실제로 보고한 상태**, **아래 줄은 우리가 내린 지령**이다.
둘이 오래 어긋나면 명령이 안 먹고 있다는 뜻이므로, 현장 디버깅에 쓰인다.

C# 쪽 `SafetyLevel` enum 도 같은 규칙이다 — `ReducedSpeed50` = 전속의 50 %.
