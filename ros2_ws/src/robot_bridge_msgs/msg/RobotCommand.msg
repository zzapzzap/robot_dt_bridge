# XAI · Unity → 통합 컨트롤러 → PLC 로 내려보내는 제어 명령.
# 버퍼1(긴급) RwrD2000~2002 · 버퍼2(정밀) RwrD3000~3002 에 대응한다.

std_msgs/Header header

string source           # 발행 주체 : "xdi" | "xag" | "unity" | "operator"
uint8 priority          # 0~255. 큰 값이 우선 (plc.yaml safety.priority 와 정합)

# --- 버퍼1 : 긴급 (배타적) ---
bool run
bool hold
bool stop

# --- 버퍼2 : 정밀 감속 (배타적) ---
bool speed_down_1       # 25 %
bool speed_down_2       # 50 %
bool speed_down_3       # 75 %

string reason           # 근거 문자열 (XAI 설명 요약 등, 로그용)
