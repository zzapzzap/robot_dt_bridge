# PLC 또는 로봇 컨트롤러에서 읽은 실제 운전 상태를 정규화한 메시지.
# header.stamp는 Jetson이 이 상태를 발행한 시각이다.
# 연결이 끊겨도 마지막 상태를 보존하므로 fresh와 age_sec를 함께 확인한다.

uint8 CONNECTION_DISCONNECTED = 0
uint8 CONNECTION_CONNECTING = 1
uint8 CONNECTION_CONNECTED = 2
uint8 CONNECTION_DEGRADED = 3

uint8 OPERATION_UNKNOWN = 0
uint8 OPERATION_MANUAL = 1
uint8 OPERATION_AUTOMATIC = 2
uint8 OPERATION_REMOTE = 3

uint8 MOTOR_UNKNOWN = 0
uint8 MOTOR_OFF = 1
uint8 MOTOR_ON = 2
uint8 MOTOR_BUSY = 3

uint8 EXECUTION_UNKNOWN = 0
uint8 EXECUTION_STOPPED = 1
uint8 EXECUTION_RUNNING = 2
uint8 EXECUTION_HOLD = 3

# 안전 신호와 고장 상태는 bool 대신 3상태로 표현한다.
# 통신 단절 시 UNKNOWN이 INACTIVE로 잘못 해석되는 것을 막기 위함이다.
uint8 SIGNAL_UNKNOWN = 0
uint8 SIGNAL_INACTIVE = 1
uint8 SIGNAL_ACTIVE = 2

std_msgs/Header header

string robot_id
uint64 sequence

uint8 connection_state
bool fresh
float32 age_sec
builtin_interfaces/Time last_controller_contact # upstream actual source의 마지막 정상 응답

uint8 operation_mode
uint8 motor_power_state
uint8 execution_state

uint8 emergency_stop_state
uint8 protective_stop_state
uint8 fault_state

# 속도는 0.0~100.0 percent 단위이다. PLC 경로의 actual은 PLC가 mirror한
# 운전속도 상태이고 Hi6 direct에서는 controller playback 비율이다. 어느 쪽도
# safety-rated 실제 축/TCP 속도 측정값이 아니다. upstream이 readback을 제공하지
# 않으면 actual_speed_valid는 false이다.
bool requested_speed_valid
float32 requested_speed_percent
bool actual_speed_valid
float32 actual_speed_percent

string fault_code
string fault_message
string status_message
