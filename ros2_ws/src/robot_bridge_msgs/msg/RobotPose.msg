# raw → degree 변환이 끝난 6축 관절각.
# Unity 는 unity_adapter 가 변환한 std_msgs/Float64MultiArray 를 구독하므로
# 이 메시지는 ROS 2 내부(기록 · XAI 입력)용이다.

std_msgs/Header header

string robot_id
string[] axis_names     # ["S","H","V","R2","B","R1"]
float64[] degrees       # 길이 6
int32[] raw             # 변환 전 원시값 (추적용)
bool calibrated         # scale/dir/offset 확정 여부
bool clamped            # limits_deg 로 잘렸는지
