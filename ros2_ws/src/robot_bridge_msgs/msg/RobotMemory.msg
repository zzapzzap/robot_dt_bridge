# PLC D 레지스터를 변환 없이 그대로 담는 원시 메시지.
# 별첨자료(robotpose_python.py / robotpose_cpp.cpp)의 필드 구성을 그대로 따른다.

std_msgs/Header header

# --- 상태 비트 (D1100.x) ---
bool run
bool hold
bool emergency_stop
bool speed_down_1
bool speed_down_2
bool speed_down_3

# --- 운전 상태 (D1000, WORD) ---
int32 operation_state

# --- 6축 축좌표 원시값 (D1002~D1012, DWORD signed) ---
#     로딩 [S,H,V,R2,B,R1] / 언로딩 [S,L,U,R,B,T]
int32 s_axis
int32 h_axis
int32 v_axis
int32 r2_axis
int32 b_axis
int32 r1_axis

# --- 통신 품질 ---
bool link_ok            # PLC 세션 정상 여부
uint32 seq              # 폴링 일련번호 (누락 검출용)
float32 read_latency_ms # 요청 → 응답 소요시간
