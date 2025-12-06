# 전체 시스템 상태
string robot_state          # IDLE, TOOL_GRIP_FRONT, WAITING_ANGLE, WORKING, TOOL_GRIP_BACK, ERROR
string current_step         # STEP_0, STEP_1, STEP_2, STEP_3, STEP_4

# 도구 정보
int32 selected_tool         # 1=걸레, 2=물뿌리개, 0=없음
string tool_name           # "걸레", "물뿌리개", "없음"

# 작업 진행 정보
float32 overall_progress    # 전체 진행률 (0-100)
float32 action_progress     # 현재 액션 진행률 (0-100)
float32 panel_angle         # 패널 각도

# 상세 상태
bool is_running            # 작업 중인지
bool waiting_for_angle     # 각도 대기 중인지
string current_action      # "WipePanel", "WaterPatrol", "None"

# 타임스탬프
builtin_interfaces/Time timestamp

# 에러 정보
bool has_error
string error_message
