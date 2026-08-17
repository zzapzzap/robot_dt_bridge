# 안전 운전 모드 — 정상운영 / 감속 3단 / 일시정지 / 비상정지 를 하나의 값으로 표현.
# 토픽 /robot/<id>/mode 로 상시 방송되고, 서비스로 설정·조회할 수 있다.

uint8 MODE_UNKNOWN = 0
uint8 MODE_RUN     = 1    # 정상 운영        속도 100 %
uint8 MODE_SLOW_1  = 2    # 감속 1 (25 % 감속)  속도  75 %
uint8 MODE_SLOW_2  = 3    # 감속 2 (50 % 감속)  속도  50 %
uint8 MODE_SLOW_3  = 4    # 감속 3 (75 % 감속)  속도  25 %
uint8 MODE_HOLD    = 5    # 일시정지 (Hold)  속도   0 %  — 전원 유지
uint8 MODE_STOP    = 6    # 비상정지         속도   0 %  — 즉시 정지

std_msgs/Header header

uint8 mode
string mode_name          # "정상 운영" · "감속 2 (50 % 감속)" · "비상정지" …
# 주의 : mode_name 의 % 는 감속률, speed_ratio 는 실제 속도 배율이다.
float32 speed_ratio       # 1.0 / 0.75 / 0.5 / 0.25 / 0.0

string source             # 이 모드를 만든 주체 : xdi | xag | unity | operator | watchdog
string reason             # 근거 (XAI 설명 요약 등)
uint8 priority            # 적용된 지령의 우선순위

bool latched              # 서비스로 고정된 상태인지 (토픽 지령은 300 ms 후 소멸)
float32 latch_remaining_s # 고정 잔여시간. 0 이면 해제될 때까지 무기한
bool link_ok              # PLC 링크 정상 여부
builtin_interfaces/Time since   # 이 모드로 진입한 시각
