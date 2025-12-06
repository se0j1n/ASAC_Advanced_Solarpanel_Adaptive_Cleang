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
coord_pub = None
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


def measure_3_points():
    from DSR_ROBOT2 import posx, movel, wait
    from DSR_ROBOT2 import task_compliance_ctrl, set_desired_force, DR_FC_MOD_ABS
    from DSR_ROBOT2 import get_tool_force, get_current_posx, DR_BASE, DR_MV_MOD_REL
    from DSR_ROBOT2 import release_force, release_compliance_ctrl, set_ref_coord, DR_TOOL

    ready = posx(541.39, -2.89, 136.5, 2.94, 135.4, 91.63)
    # ready = posx(380.3, 0.31, 300.61, 179.05, -136.43, -90.49)
    # ready = posx(300.58, 1.95, 340.75, 177.73, -146.6, -91.42)
    # 그립 위치
    # 변경 전
    # tool_position = posx([294.02, -301.17, 280.19-228.6, 40.53, -179.75, 40.87])
    # tool_approach = posx([294.02, -301.17, 449-228.6, 40.53, -179.75, 40.87])

    # 변경 후
    tool_position = posx([258.57, -294.36, 282.14-228.6, 4.38, -174.93, 91.16])
    tool_approach = posx([258.57, -294.36, 449-228.6, 4.38, -174.93, 91.16])


    move_1 = posx(0,-50,0,0,0,0)
    move_2 = posx(-60,0,-60,0,0,0)
    move_next = [move_1, move_2]


        # 그립 이동
    gripper_open()
    movel(tool_approach, vel=VELOCITY, acc=ACC)
    movel(tool_position, vel=VELOCITY, acc=ACC)
    gripper_close()
    movel(tool_approach, vel=VELOCITY, acc=ACC)

    pts = []

    print("찌르는 위치로 이동")
    movel(ready, v=100, a=50)

    f0 = average_force()

    for i in range(3):

        print("찌르기 시작")

        # task_compliance_ctrl()
        # set_desired_force([30,0,0,0,0,0],[1,0,0,0,0,0],mod=DR_FC_MOD_ABS)
        wait(0.2)

        # set_ref_coord(DR_TOOL)
        while True:
            fx, fy, fz = get_tool_force(DR_TOOL)[0:3]
            d_fx = fx - f0[0]
            d_fz = fz - f0[2]

            pos_raw = get_current_posx(ref=DR_BASE)
            if pos_raw is None:
                continue

            pos = pos_raw[0]     # ★ 좌표 6개 리스트만 사용해야 한다!

            if d_fz < -3:
                wait(0.1)
                # set_ref_coord(DR_BASE)
                # wait(0.5)
                pts.append(pos)
                print(f"[접촉 {i+1}] {pos}")
                break

            movel(posx(2,0,0,0,0,0), v=200, a=100, mod=DR_MV_MOD_REL)

        # release_force()
        # release_compliance_ctrl()

        
        
        if i < 2:
            movel(posx(-10,0,0,0,0,0), v=200, a=100, mod=DR_MV_MOD_REL)
            movel(move_next[i], v=100, a=50, mod=DR_MV_MOD_REL)

        wait(1)

    return pts


def average_force():
    from DSR_ROBOT2 import movel, posx
    from DSR_ROBOT2 import get_tool_force, get_current_posx, DR_BASE, DR_MV_MOD_REL, DR_TOOL

    n = 5
    f_sum = [0.0, 0.0, 0.0]
    for i in range(n):

        # 1) 일정 거리 이동 (상대 이동)
        movel(posx(2,0,0,0,0,0), v=200, a=100, mod=DR_MV_MOD_REL)

        # 2) Force 측정
        fx, fy, fz = get_tool_force(DR_TOOL)[0:3]

        # 3) Force 누적
        f_sum = [
            f_sum[0] + fx,
            f_sum[1] + fy,
            f_sum[2] + fz
        ]

        # Debug 출력
        print(f"[DEBUG] {i+1}/{n}번째 측정 force = "
            f"({fx:.3f}, {fy:.3f}, {fz:.3f}) → 누적값 = {f_sum}")

    # 4) 평균 force 계산
    f_avg = [
        f_sum[0] / n,
        f_sum[1] / n,
        f_sum[2] / n
    ]
    return f_avg


def calculate_coord(points):
    from DSR_ROBOT2 import calc_coord, DR_BASE, wait
    x1, x2, x3 = points

    pose = calc_coord(x1, x2, x3, ref=DR_BASE, mod=0)

    print("[Node1] 계산된 좌표계 =", pose)
    wait(2)
    return pose


def publish_coord(pose):
    global coord_pub
    msg = String()
    msg.data = str(pose)
    coord_pub.publish(msg)
    print("[Node1] 좌표계 Publish :", msg.data)


def main(args=None):
    global node, coord_pub, status_pub, force_pub, start_received, already_executed

    rclpy.init(args=args)
    node = rclpy.create_node("coord_calculator_node", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    coord_pub = node.create_publisher(String, "calculated_coord", 10)
    status_pub = node.create_publisher(String, "robot_status", 10)
    force_pub = node.create_publisher(String, "force_data", 10)

    node.create_subscription(String, "start_signal", start_signal_callback, 10)

    initialize_robot()

    print("\n=== Node1: 좌표계 계산 노드 실행 중 ===")

    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)

        if start_received and not already_executed:
            publish_status("작업 시작")

            pts = measure_3_points()
            print(pts, "전체 좌표계 : ")
            pose = calculate_coord(pts)
            publish_coord(pose)
            publish_status("좌표 계산 완료")

            already_executed = True     # ★ 한 번만 실행되도록 설정
            start_received = False

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
    

if __name__ == "__main__":
    main()
