# 안전 운전 모드.
#
# 이름에 결과 속도를 그대로 박아 넣는다 — REDUCED_SPEED_50 = 전속의 50 % 로 운전.
# 「감속 N」 표기는 감속률/잔여속도가 갈려 정반대 해석이 가능하므로 쓰지 않는다.
# 정지 용어는 ISO 10218 / ISO-TS 15066 을 따른다.
#
# 토픽 /robot/<id>/mode 로 상시 방송되고, 서비스로 설정·조회할 수 있다.

uint8 MODE_UNKNOWN         = 0
uint8 MODE_NORMAL          = 1    # 정상 운전 · 전속       속도 100 %
uint8 MODE_REDUCED_75      = 2    # 속도제한 75 %          속도  75 %   ← PLC speed_down_1
uint8 MODE_REDUCED_50      = 3    # 속도제한 50 %          속도  50 %   ← PLC speed_down_2
uint8 MODE_REDUCED_25      = 4    # 속도제한 25 %          속도  25 %   ← PLC speed_down_3
uint8 MODE_PROTECTIVE_STOP = 5    # 보호정지 — 전원 유지    속도   0 %   ← PLC hold
uint8 MODE_EMERGENCY_STOP  = 6    # 비상정지               속도   0 %   ← PLC stop

std_msgs/Header header

uint8 mode
string mode_name          # 정규 식별자 : "REDUCED_SPEED_50" · "EMERGENCY_STOP" …
string mode_label         # 사람이 읽는 라벨 : "속도제한 50 %" · "비상정지" …
float32 speed_ratio       # 전속 대비 속도 배율 — 1.0 / 0.75 / 0.5 / 0.25 / 0.0
                          # 기계 판독은 항상 이 값을 쓴다 (이름 문자열 파싱 금지)

string source             # 이 모드를 만든 주체 : xdi | xag | unity | operator | watchdog
string reason             # 근거 (XAI 설명 요약 등)
uint8 priority            # 적용된 지령의 우선순위

bool latched              # 서비스로 고정된 상태인지 (토픽 지령은 300 ms 후 소멸)
float32 latch_remaining_s # 고정 잔여시간. 0 이면 해제될 때까지 무기한
bool link_ok              # PLC 링크 정상 여부
builtin_interfaces/Time since   # 이 모드로 진입한 시각
