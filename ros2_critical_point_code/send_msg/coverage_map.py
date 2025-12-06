#!/usr/bin/env python3
import rclpy
import DR_init
from std_msgs.msg import String
import numpy as np
import json   # coverage map 송신용 JSON 변환에 사용

# ============================================
# 1. 로봇 기본 설정
# ============================================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TCP = "Tool Weight"
ROBOT_TOOL = "GripperDA_v1"

# DR_init에 로봇 ID/모델 등록 (ROS2 서비스 사용 위해 필수)
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# ============================================
# 2. Coverage Map 설정
# ============================================
WALL_WIDTH_MM = 400          # 벽 가로 길이(mm)
WALL_HEIGHT_MM = 300         # 벽 세로 길이(mm)
RESOLUTION_MM = 2            # 1 grid = 2mm

GRID_W = int(WALL_WIDTH_MM / RESOLUTION_MM)  # grid 가로 픽셀 수
GRID_H = int(WALL_HEIGHT_MM / RESOLUTION_MM) # grid 세로 픽셀 수

# coverage map (float32 사용)
coverage = np.zeros((GRID_H, GRID_W), dtype=np.float32)

# 중앙을 (0,0)으로 보기 위한 오프셋
CENTER_U = GRID_W // 2
CENTER_V = GRID_H // 2

# 브러시 물리 크기를 grid 픽셀 단위로 환산
BRUSH_X_MM = 20
BRUSH_Y_MM = 135

HALF_X_PX = int((BRUSH_X_MM / 2) / RESOLUTION_MM)
HALF_Y_PX = int((BRUSH_Y_MM / 2) / RESOLUTION_MM)

# 이동 판정
MOVE_THRESHOLD = 1

# 로봇 이동 기록용 이전 grid 좌표
last_u = None
last_v = None

# ROS2 Node 관련 변수들
node = None
ratio_pub = None
coverage_pub = None
start_received = False

# 사용자 정의 좌표계를 생성할 초기 pose(하드코딩)
coord = [
    416.350341796875+10,
    -15.415404319763184,
    286.5602111816406-10,
    177.66575622558594,
    -135.9037322998047,
    89.92499542236328
]

# 사용자 좌표계 ID
USER_COORD_ID = None


# ============================================
# 3-A. ★★★ get_current_posx 보호 Wrapper 추가 ★★★
# ============================================
def safe_get_current_posx(max_retry=10):
    """
    get_current_posx()가 None, 빈 리스트, pose 길이가 3 미만,
    또는 예외 발생 시 자동으로 재시도한다.
    움직이는 동안 SDK가 pose를 안정적으로 못 줄 때 대비.
    """
    from DSR_ROBOT2 import get_current_posx
    import time

    for attempt in range(1, max_retry + 1):
        try:
            res = get_current_posx()
        except Exception as e:
            print(f"[WARN] get_current_posx() 예외 발생 (시도 {attempt}/{max_retry}): {e}")
            time.sleep(0.05)
            continue

        # 정상 구조: pose가 최소 3개 이상일 것
        if (
            isinstance(res, (list, tuple)) and
            len(res) >= 2 and
            isinstance(res[0], (list, tuple)) and
            len(res[0]) >= 3
        ):
            pose = res[0]
            print(f"[POSE OK] x={pose[0]:.3f} y={pose[1]:.3f} z={pose[2]:.3f} (len={len(pose)})")
            return res

        print(f"[POSE BAD] res={res} (pose 길이 부족 또는 구조 불량)")
        time.sleep(0.15)

    print("[FATAL] get_current_posx() 여러 번 실패 → None 반환")
    return None


# ============================================
# 3-B. 사용자 좌표계 생성 함수
# ============================================
def create_user_coord_safely(coord_posx, max_retry=5):
    """
    set_user_cart_coord()가 실패하는 경우 자동 재시도.
    좌표계 생성 직후 0.5초 대기하여 안정화.
    """
    from DSR_ROBOT2 import set_user_cart_coord, get_current_posx
    import time

    for attempt in range(1, max_retry + 1):
        current = get_current_posx()
        if current is None:
            print(f"[ERROR] get_current_posx() 실패 → {attempt}번째 재시도")
            time.sleep(0.2)
            continue

        current_ref = current[1]
        USER_ID = set_user_cart_coord(coord_posx, ref=current_ref)

        if USER_ID is None or USER_ID < 1:
            print(f"[WARN] 좌표계 생성 실패 → {attempt}/{max_retry} 재시도")
            time.sleep(0.3)
            continue

        print(f"[OK] 사용자 좌표계 생성 완료: ID = {USER_ID}")
        time.sleep(0.5)
        return USER_ID

    print("[FATAL] 사용자 좌표계 생성 완전 실패")
    return None


# ============================================
# 4. Coverage Map 변환 및 기록 함수들
# ============================================
def world_to_grid(x_mm, y_mm):
    """ 실제(mm) 좌표 → grid(u,v) 좌표 변환 """
    u = int(x_mm / RESOLUTION_MM) + CENTER_U
    v = int(y_mm / RESOLUTION_MM) + CENTER_V
    return u, v


def mark_rect_coverage(u, v):
    """ 브러시 크기만큼 사각형 영역을 coverage map에 +0.1 추가 """
    for i in range(-HALF_X_PX, HALF_X_PX + 1):
        for j in range(-HALF_Y_PX, HALF_Y_PX + 1):
            uu = u + i
            vv = v + j
            if 0 <= uu < GRID_W and 0 <= vv < GRID_H:
                coverage[GRID_H - 1 - vv][uu] += 0.1


def update_coverage_from_robot():
    """
    로봇 현재 위치를 받아 coverage map 업데이트 수행.
    움직임이 일정 threshold 이상일 때만 기록.
    """
    global last_u, last_v, USER_COORD_ID

    if USER_COORD_ID is None:
        print("[ERROR] USER_COORD_ID is None → get_current_posx 불가능")
        return

    # ★★★ 기존 get_current_posx → safe_get_current_posx 로 교체 ★★★
    result = safe_get_current_posx()
    if result is None:
        print("[ERROR] safe_get_current_posx() 실패 → 이번 루프 스킵")
        return

    pos = result[0]   # pose list
    x_mm = pos[0]
    y_mm = pos[1]
    z_mm = pos[2]

    if z_mm < -100:
        return

    u, v = world_to_grid(x_mm, y_mm)

    if last_u is None:
        mark_rect_coverage(u, v)
        last_u, last_v = u, v
        return

    if abs(u - last_u) < MOVE_THRESHOLD and abs(v - last_v) < MOVE_THRESHOLD:
        return

    mark_rect_coverage(u, v)
    last_u, last_v = u, v


def get_coverage_ratio():
    """ coverage map 중 value>0인 grid 비율 계산 """
    return np.sum(coverage > 0) / coverage.size


# ============================================
# 5. ROS Node (main)
# ============================================
def main(args=None):
    global node, ratio_pub, coverage_pub, start_received

    # ROS node 생성
    rclpy.init(args=args)
    node = rclpy.create_node("coverage_cleaner", namespace=ROBOT_ID)

    # DSR 로봇 서비스 연결을 위해 node 등록
    DR_init.__dsr__node = node

    # publisher 생성
    ratio_pub = node.create_publisher(String, "coverage_ratio", 10)
    coverage_pub = node.create_publisher(String, "coverage_map", 10)

    # START 신호 구독
    node.create_subscription(String, "start_cleaning", start_signal_callback, 10)

    # 로봇 초기 설정 + 사용자 좌표계 생성
    initialize_robot()

    print("\n=== Coverage Cleaner Node 실행 중 ===")

    # main loop
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)

        if start_received:
            update_coverage_from_robot()
            publish_ratio(get_coverage_ratio())
            publish_coverage_map()


# ============================================
# 6. 로봇 초기화 함수
# ============================================
def initialize_robot():
    """ Tool/TCP 설정 후 사용자 좌표계 생성 """
    global USER_COORD_ID
    from DSR_ROBOT2 import set_tool, set_tcp, posx, wait

    print("=== Robot Init ===")
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)

    # 좌표계 생성용 Pose를 posx 형태로 변환
    x, y, z, rx, ry, rz = coord
    coord_posx = posx(x, y, z, rx, ry, rz)

    # ★★★★★ 좌표계 생성 실패 시 재시도하도록 변경 ★★★★★
    while True:
        USER_COORD_ID = create_user_coord_safely(coord_posx)

        if USER_COORD_ID is not None:
            # 성공
            print(f"[INIT] 사용자 좌표계 사용 ID = {USER_COORD_ID}")
            break

        # 실패 → 재시도
        print("[ERROR] 사용자 좌표계 생성 실패 → 1초 후 재시도")
        wait(1.0)

    print("===================")


# ============================================
# 7. ROS Publish & Callback
# ============================================
def start_signal_callback(msg):
    """ 외부에서 START 신호 수신 시 기록 시작 """
    global start_received
    start_received = True
    print("[Node] START → Coverage 기록 시작")


def publish_ratio(ratio):
    """ coverage 비율 publish """
    m = String()
    m.data = f"{ratio:.3f}"
    ratio_pub.publish(m)


def publish_coverage_map():
    """ coverage map 전체(JSON) publish """
    global coverage_pub
    json_str = json.dumps(coverage.tolist())
    m = String()
    m.data = json_str
    coverage_pub.publish(m)


# ============================================
# 8. Entry Point
# ============================================
if __name__ == "__main__":
    main()
