# 데이터 계약

## PLC → ROS 2

로봇별 PLC-local register map은 `config/robots.yaml`에서 정의한다. 공개 gateway는
각 PLC sample을 다음으로 동시에 변환한다.

| 토픽 | 타입 | 계약 |
|---|---|---|
| `/robot/<id>/pose` | `RobotPose` | controller 6축 degree, PLC raw 포함 |
| `/robot/<id>/cmd_degs_raw` | `RobotPose` | 위 controller pose와 같은 sample, Unity adapter 입력 |
| `/robot/<id>/joint_states` | `sensor_msgs/JointState` | CAD/URDF 보정이 적용된 radian 값 |
| `/robot/<id>/memory` | `RobotMemory` | PLC raw/status/link 진단 |
| `/robot/<id>/control_state` | `RobotControlState` | No.9~17 raw command/register readback |
| `/robot/<id>/status` | `RobotStatus` | link/freshness; unmapped actual fields are UNKNOWN |
| `/robot/<id>/mode` | `SafetyMode` | actual feedback을 legacy mode로 정규화 |

Unity adapter는 다음 표준 메시지를 만든다.

| 토픽 | 타입 | 배열 |
|---|---|---|
| `/robot/<id>/cmd_degs` | `Float64MultiArray` | CAD/Unity 보정이 적용된 degree 6개 |
| `/robot/<id>/state` | `Int32MultiArray` | `[run,hold,estop,sd1,sd2,sd3,op_state]` |
| `/robot/<id>/mode_unity` | `Int32MultiArray` | `[mode,speed%,valid,latched]` |

통신 단절에서는 마지막 상태를 정상으로 반복하지 않는다. `RobotStatus.fresh=false`,
signal은 `UNKNOWN`이며 legacy Unity state는 watchdog이 만료되도록 억제한다.

## ROS service → PLC

공개 운영 경로는 `/unity_command`와 `RobotCommand`를 PLC에 전달하지 않는다.
명령은 service-only다.

| 서비스 | 역할 |
|---|---|
| `/robot/<id>/get_status` | cache 또는 강제 PLC actual read |
| `/robot/<id>/set_speed_percent` | D1016/18/20 scalar 25/50/75/100 request |
| `/robot/<id>/set_hold` | D1100.0 masked set/clear |
| `/robot/<id>/trigger_action` | D1100.2~5 configured pulse |

응답의 `accepted`, `controller_ack`, `confirmed`를 서로 바꾸어 해석하지 않는다.

- `accepted`: config/policy와 입력값 검증 성공
- `controller_ack`: MC write 응답의 end code가 정상
- `confirmed`: 후속 PLC actual/ack가 목표 상태와 일치
- `register_readback`: command register가 요청값 또는 pulse-clear 값과 일치

command register를 그대로 다시 읽은 echo는 actual robot feedback이 아니다.
따라서 현재 speed service는 정상 write/readback 뒤에도 `confirmed=false`와
`ACTUAL_FEEDBACK_UNAVAILABLE`을 반환한다.

## 축과 단위

현재 확정된 현대 로딩 표의 축은 `[S,H,V,R2,B,R1]`이다. 언로딩의 고유 PLC map과
축 계약은 아직 없다. 기존 `[S,L,U,R,B,T]`는 야스카와 자료이므로 현대 로봇
계약으로 사용하지 않는다.

```text
controller_deg[i] = raw_dword[i] * scale[i] * dir[i] + offset[i]
visual_deg[i] = controller_deg[i] * visual_dir[i] + visual_offset[i]
joint_state_rad[i] = visual_deg[i] * pi / 180
```

`/pose.degrees`는 controller 좌표, `/joint_states`, `/cmd_degs`는
CAD/URDF 좌표다. 따라서 시각화 부호·영점 보정이 PLC 원본 값을
바꾸지 않는다.

DWORD는 PLC 계약의 low/high word order를 명시적으로 확인한다. `calibrated=false`인
동안 RViz/Unity 값은 연계 시험용이고 정밀 위치나 안전 판단에 사용하지 않는다.

## 안전 경계

PLC의 D1100.1 E-stop 표기 bit는 읽기 전용이며 Jetson이 해제하거나 생성하지 않는다.
legacy ROS `request_stop`도 safety-rated E-stop이 아니다. D1100.1에는 쓰기 API가
없다. 링크 단절 시 Jetson 명령은 전달될 수 없으므로 PLC-side heartbeat watchdog과
독립 안전회로가 필요하다.
