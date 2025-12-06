#!/usr/bin/env python3
import rclpy
import DR_init
from std_msgs.msg import String


ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TCP = "Tool Weight"
ROBOT_TOOL = "GripperDA_v1"
VELOCITY = 120
ACC = 60

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

node = None
theta_pub = None
status_pub = None
force_pub = None
start_received = False
already_executed = False    


def initialize_robot():
    from DSR_ROBOT2 import set_tool, set_tcp, set_velx, set_accx

    print("=== Robot Init ===")
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)
    set_velx(VELOCITY)
    set_accx(ACC)
    print("===================\n")


def start_signal_callback(msg):
    global start_received, already_executed

    # ★ 이미 한 번 실행한 상태라면 START 신호 무시
    if already_executed:
        return

    if msg.data == "START" and not start_received:
        print("[Node1] START 신호 수신 → 좌표계 생성 시작")
        start_received = True


def publish_status(msg):
    global status_pub
    m = String()
    m.data = msg
    status_pub.publish(m)
    print("[Node1 Status]", msg)


def publish_force(d_fx, d_fz):
    global force_pub
    m = String()
    m.data = f"{d_fx:.3f}, {d_fz:.3f}"
    force_pub.publish(m)



# ★★★★★ safe_get_current_posx: SDK 오류 + 비정상 구조 완전 방어 ★★★★★
def safe_get_current_posx(max_retry: int = 10):
    from DSR_ROBOT2 import get_current_posx
    import time

    for attempt in range(1, max_retry + 1):
        try:
            res = get_current_posx()
        except Exception as e:
            print(f"[WARN] get_current_posx() 예외 발생 (시도 {attempt}/{max_retry}): {e}")
            time.sleep(0.05)
            continue

        # 정상 구조:
        #   ([x,y,z,rx,ry,rz], solution)
        if (
            isinstance(res, (list, tuple))       # 전체는 list/tuple 가능
            and len(res) >= 2                    # pose + sol
            and isinstance(res[0], (list, tuple))# pose 리스트
            and len(res[0]) == 6                 # pose 길이는 6
        ):
            return res

        print(f"[WARN] get_current_posx() 비정상 응답 (시도 {attempt}/{max_retry}): {res}")
        time.sleep(0.05)

    print("[FATAL] get_current_posx()가 여러 번 실패했습니다. None 반환.")
    return None
# ★★★★★ safe_get_current_posx 끝 ★★★★★



def measure_2_points():
    from DSR_ROBOT2 import posx, movel, wait
    from DSR_ROBOT2 import task_compliance_ctrl, set_desired_force, DR_FC_MOD_ABS
    from DSR_ROBOT2 import get_tool_force, DR_BASE, DR_MV_MOD_REL
    from DSR_ROBOT2 import release_force, release_compliance_ctrl

    # 그립 위치
    tool_position = posx([258.57, -294.36, 282.14, 4.38, -174.93, 91.16])
    tool_approach = posx([258.57, -294.36, 450, 4.38, -174.93, 91.16])
    
    # 그립 후 준비자세
    center = posx([500.47, -140.41, 400.64, 155.36, -179.81, 155.21])

    # 찌르기 준비 위치
    ready = posx(380.3, -15.45, 300.51, 2.94, 135.4, 91.63)

    # 밑으로 이동
    move_1 = posx(-80,0,-60,0,0,0)

    # 그립 이동
    gripper_open()
    movel(tool_approach, vel=VELOCITY, acc=ACC)
    movel(tool_position, vel=VELOCITY, acc=ACC)
    gripper_close()
    movel(tool_approach, vel=VELOCITY, acc=ACC)
    movel(center, vel=VELOCITY, acc=ACC)

    pts = []

    print("찌르는 위치로 이동")
    movel(ready, v=100, a=50)

    for i in range(2):

        print("찌르기 시작")

        f0 = get_tool_force()[0:3]
        wait(0.2)

        while True:
            fx, fy, fz = get_tool_force()[0:3]
            d_fx = fx - f0[0]
            d_fz = fz - f0[2]

            # ★ 기존 호출 삭제 → wrapper로 대체
            pos_raw = safe_get_current_posx()

            # 완전 실패 시 skip
            if pos_raw is None:
                print("[ERROR] safe_get_current_posx() -> None, 이번 루프 스킵")
                continue

            # ============================
            #   DEBUG + 안전 검사
            # ============================

            # 1) None
            if pos_raw is None:
                print("[WARN] get_current_posx() returned None")
                continue

            # 2) 최소 길이
            if len(pos_raw) < 1:
                print("[ERROR] get_current_posx() returned empty list")
                continue

            pos = pos_raw[0]

            # 3) pose 길이 검사
            if len(pos) != 6:
                print(f"[ERROR] Invalid pose length: {len(pos)}, pose={pos}")
                continue

            # ============================
            #   안전 검사 끝
            # ============================


            if d_fx < -3 or d_fz > 3:
                wait(0.1)
                pts.append(pos)
                print(f"[접촉 {i+1}] {pos}")
                break

            movel(posx(2,0,0,0,0,0), v=200, a=100, mod=DR_MV_MOD_REL)

        if i < 1:
            movel(move_1, v=100, a=50, mod=DR_MV_MOD_REL)


    ### 후반부 ###
    movel(tool_approach, vel=VELOCITY, acc=ACC)
    movel(tool_position, vel=VELOCITY, acc=ACC)
    gripper_open()
    movel(tool_approach, vel=VELOCITY, acc=ACC)

    return pts




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



def calculate_theta(points):
    from DSR_ROBOT2 import  wait
    from math import atan2
    x1, x2  = points

    dx = x1[0]-x2[0]
    dz = x1[2]-x2[2]

    theta_rad = atan2(dz,dx)
    theta = theta_rad * 180.0 / 3.141592653589793
    print("[Node1] 계산된 각도 =", theta)
    wait(2)
    return theta


def publish_theta(theta):
    global theta_pub
    msg = String()
    msg.data = str(theta)
    theta_pub.publish(msg)
    print("[Node1] 좌표계 Theta :", msg.data)


def main(args=None):
    global node, theta_pub, status_pub, force_pub, start_received, already_executed

    rclpy.init(args=args)
    node = rclpy.create_node("coord_calculator_node", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    theta_pub = node.create_publisher(String, "calculated_coord", 10)
    status_pub = node.create_publisher(String, "robot_status", 10)
    force_pub = node.create_publisher(String, "force_data", 10)

    node.create_subscription(String, "start_signal", start_signal_callback, 10)

    initialize_robot()

    print("\n=== Node1: 좌표계 계산 노드 실행 중 ===")

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)

        if start_received and not already_executed:
            publish_status("작업 시작")

            pts = measure_2_points()
            print(pts, "전체 좌표계 : ")
            theta = calculate_theta(pts)
            publish_theta(theta)
            publish_status("좌표 계산 완료")

            already_executed = True     # ★ 한 번만 실행되도록 설정
            start_received = False


if __name__ == "__main__":
    main()
