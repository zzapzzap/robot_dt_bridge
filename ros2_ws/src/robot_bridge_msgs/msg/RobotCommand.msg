# XAI · Unity → 통합 컨트롤러 → PLC 로 내려보내는 제어 명령.
# 버퍼1(긴급) RwrD2000~2002 · 버퍼2(정밀) RwrD3000~3002 에 대응한다.

std_msgs/Header header

string source           # 발행 주체 : "xdi" | "xag" | "unity" | "operator"
uint8 priority          # 0~255. 큰 값이 우선 (plc.yaml safety.priority 와 정합)

# --- 버퍼1 : 긴급 (배타적) ---
bool run
bool hold
bool stop

# --- 버퍼2 : 속도제한 (배타적) ---
# 필드명은 에이시스 사양서(감속률 표기)를 그대로 따르고,
# 모드 값은 결과 속도로 부른다. 숫자 방향이 반대이므로 주의.
bool speed_down_1       # 25 % 감속  →  MODE_REDUCED_75  (속도 75 %)
bool speed_down_2       # 50 % 감속  →  MODE_REDUCED_50  (속도 50 %)
bool speed_down_3       # 75 % 감속  →  MODE_REDUCED_25  (속도 25 %)

string reason           # 근거 문자열 (XAI 설명 요약 등, 로그용)
