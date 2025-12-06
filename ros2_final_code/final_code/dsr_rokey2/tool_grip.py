#!/usr/bin/env python3
import rclpy
import DR_init
from std_msgs.msg import Int32

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA"

VELOCITY = 120
ACC = 60

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# 전역 상태
selected_tool_id = 0          # 현재 선택된 툴 (1: 걸레, 2: 물뿌리개)
new_front_command = False     # 전반부 시작 명령 플래그 (/tool_grip_start)
task_end_signal = False       # 후반부 시작 플래그 (/task_end)


def tool_grip_start_callback(msg):
    """⭐ main_controller로부터 전반부 시작 명령 수신"""
    global selected_tool_id, new_front_command
    selected_tool_id = msg.data
    new_front_command = True
    print(f"[Tool Grip Start] tool 번호 수신: {selected_tool_id}")


def task_end_callback(msg):
    """⭐ main_controller로부터 후반부 시작(/task_end) 수신"""
    global task_end_signal
    task_end_signal = True
    print(f"[TASK END] 신호 수신")


def initialize_robot():
    from DSR_ROBOT2 import set_tool, set_tcp
    print("Initializing robot...")
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)


def gripper_close():
    from DSR_ROBOT2 import set_digital_output, wait, ON, OFF
    set_digital_output(1, ON)
    set_digital_output(2, OFF)
    wait(0.5)


def gripper_open():
    from DSR_ROBOT2 import set_digital_output, wait, ON, OFF
    set_digital_output(1, OFF)
    set_digital_output(2, ON)
    wait(0.5)


def get_positions(tool_id):
    """툴별 위치 정의 공통 함수"""
    from DSR_ROBOT2 import posx

    # 로봇이 작업 중 모을 중앙 위치 (툴을 들고 있을 때 서 있는 곳)
    center = posx([500.47, -140.41, 400.64, 155.36, -179.81, 155.21])

    if tool_id == 1:  # 걸레
        tool_position = posx([313.22, -250.57, 268.43, 5.31, -175.18, 92.17])
        tool_approach = posx([313.22, -250.57, 450.0, 5.31, -175.18, 92.17])
    elif tool_id == 2:  # 물뿌리개
        tool_position = posx([299.67, -184.33, 265.68, 5.31, -175.18, 92.17])
        tool_approach = posx([299.67, -184.33, 450.0, 5.31, -175.18, 92.17])
    else:
        print("[ERROR] 잘못된 tool 번호.")
        return None, None, None

    return center, tool_position, tool_approach


def perform_front(tool_id, pub_done):
    """
    전반부: 툴 잡으러 가서 집고 center로 돌아오는 동작
    끝나면 /task_done = 1 을 publish 하고 return.
    """
    from DSR_ROBOT2 import movel

    print(f"\n[FRONT] TOOL {tool_id} 전반부 작업 시작")

    center, tool_position, tool_approach = get_positions(tool_id)
    if center is None:
        return False

    # 안전하게 시작: 그리퍼를 먼저 연다
    gripper_open()

    # 툴 접근 → 툴 위치 → 그리퍼 닫기 → 다시 접근 높이 → center로 이동
    movel(tool_approach, vel=VELOCITY, acc=ACC)
    movel(tool_position, vel=VELOCITY, acc=ACC)
    gripper_close()
    movel(tool_approach, vel=VELOCITY, acc=ACC)
    movel(center, vel=VELOCITY, acc=ACC)

    print("=== 전반부 완료 ===")

    # 전반부 완료 신호
    msg = Int32()
    msg.data = 1  # 전반부 완료
    pub_done.publish(msg)
    print("[PUB] /task_done = 1 (전반부 완료)")

    return True


def perform_back(tool_id, pub_done):
    """
    후반부: center에서 툴 보관 위치로 가져다 놓고 돌아오는 동작
    끝나면 /task_done = 2 를 publish 하고 return.
    """
    from DSR_ROBOT2 import movel

    print(f"\n[BACK] TOOL {tool_id} 후반부 작업 시작")

    center, tool_position, tool_approach = get_positions(tool_id)
    if center is None:
        return False

    # center(툴 들고 있는 위치) → 툴 접근 → 툴 위치 → 그리퍼 열기 → 다시 접근 높이 등등
    # 현재 center에서 시작한다고 가정
    movel(tool_approach, vel=VELOCITY, acc=ACC)
    movel(tool_position, vel=VELOCITY, acc=ACC)
    gripper_open()
    movel(tool_approach, vel=VELOCITY, acc=ACC)
    # 필요하면 center로 돌아올 수도 있음 (지금은 생략하거나 필요에 따라 추가)
    # movel(center, vel=VELOCITY, acc=ACC)

    print("=== 후반부 완료 ===")

    msg = Int32()
    msg.data = 2  # 후반부 완료
    pub_done.publish(msg)
    print("[PUB] /task_done = 2 (후반부 완료)")

    return True


def main(args=None):
    global new_front_command, task_end_signal, selected_tool_id

    rclpy.init(args=args)
    node = rclpy.create_node("tool_handler_node", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    pub_done = node.create_publisher(Int32, "/task_done", 10)

    # /tool_grip_start: 전반부 시작 명령
    node.create_subscription(Int32, "/tool_grip_start", tool_grip_start_callback, 10)
    # /task_end: 후반부 시작 명령
    node.create_subscription(Int32, "/task_end", task_end_callback, 10)

    initialize_robot()

    print("\n[READY] /tool_grip_start, /task_end 명령 대기 중...\n")

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)

            # 전반부 시작 요청 처리
            if new_front_command:
                tool = selected_tool_id
                new_front_command = False

                print(f"[MAIN] 전반부 수행 시작 (tool={tool})")
                perform_front(tool, pub_done)
                print("\n[WAITING] /task_end 또는 다음 /tool_grip_start 대기...\n")

            # 후반부 시작 요청 처리
            if task_end_signal:
                tool = selected_tool_id   # 마지막에 사용한 툴 기준으로
                task_end_signal = False

                print(f"[MAIN] 후반부 수행 시작 (tool={tool})")
                perform_back(tool, pub_done)
                print("\n[WAITING] 다음 /tool_grip_start 대기...\n")

    except KeyboardInterrupt:
        print("\n종료됨")

    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
