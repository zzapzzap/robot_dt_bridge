# PLC 명령 레지스터를 다시 읽어 정규화한 상태.
# 이 값은 D1016/D1018/D1020 및 D1100의 command/register readback이며,
# 로봇 컨트롤러가 실제 동작을 수행했다는 피드백은 아니다.

std_msgs/Header header

string robot_id
uint64 sequence
bool fresh
float32 age_sec

# No.9~11: 감속 명령 레지스터의 원시 WORD 값
uint16 slowdown_25_raw
uint16 slowdown_50_raw
uint16 slowdown_75_raw

# No.12~17: D1100 전체 WORD와 개별 비트
uint16 control_word_raw

bool slowdown_25_active
bool slowdown_50_active
bool slowdown_75_active

bool hold
bool emergency_stop
bool fault_reset
bool device_home
bool robot_home
bool standby
