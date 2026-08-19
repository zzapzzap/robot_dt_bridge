# PLC raw에 scale을 적용한 controller 기준 6축 좌표.
# CAD 부호/영점 보정은 이 메시지를 바꾸지 않고 RViz joint_states와
# Unity cmd_degs를 만들 때만 별도로 적용한다.

std_msgs/Header header

string robot_id
string[] axis_names     # config에서 확정한 6축 이름 (Hi6 기본 J1..J6)
float64[] degrees       # controller 좌표(deg), 길이 6
int32[] raw             # 변환 전 원시값 (추적용)
bool calibrated         # controller/visual 변환의 현장 FAT 완료 여부
bool clamped            # limits_deg 로 잘렸는지
