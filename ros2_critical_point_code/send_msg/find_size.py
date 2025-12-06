#!/usr/bin/env python3
import rclpy
import DR_init
from std_msgs.msg import String
from ast import literal_eval   # 문자열 리스트 파싱용

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TCP = "Tool Weight"
ROBOT_TOOL = "GripperDA_v1"
VELOCITY = 50
ACC = 50

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

node = None
force_pub = None
coord_sub = None
new_coord = None
panel_pub = None

received_coord = None          # 받은 좌표계 값
executed_once = False          # 전체 동작 한 번만 실행
move_requested = False         # 콜백에서 "이제 움직여도 됨" 신호


# =========================================
#                  main
# =========================================
def main(args=None):
    global node, force_pub, coord_sub
    global move_requested
    global new_coord, panel_pub

    rclpy.init(args=args)
    node = rclpy.create_node("coord_move_subscriber", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    # force publish
    force_pub = node.create_publisher(String, "force_data", 10)

    # new 좌표계
    new_coord = node.create_publisher(String, "new_coord", 10)

    # 패널 데이터
    panel_pub = node.create_publisher(String, "panel_data", 10)



    # 좌표계 subscribe
    coord_sub = node.create_subscription(
        String,
        "calculated_coord",
        coord_callback,
        10
    )

    initialize_robot()

    print("\n=== 좌표계 수신 대기 중... ===")

    # ❗ rclpy.spin(node) 대신, spin_once + 메인 루프에서 동작 실행
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)

        # 콜백에서 move_requested = True로 바꾸면 여기서 실제 동작
        if move_requested and (not executed_once) and (received_coord is not None):
            print("[Main] 좌표계 수신 완료 → move_using_coord 실행")
            run_motion_once()

    rclpy.shutdown()


def initialize_robot():
    from DSR_ROBOT2 import set_tool, set_tcp, set_velx, set_accx
    print("=== Robot Init ===")
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)
    set_velx(VELOCITY)
    set_accx(ACC)
    print("===================\n")


# =========================================
#          좌표계 수신 콜백
# =========================================
def coord_callback(msg):
    global received_coord, move_requested, executed_once

    print(f"[Callback] 좌표계 수신: {msg.data}")

    try:
        received_coord = literal_eval(msg.data)
        print("[Callback] 파싱된 좌표계 =", received_coord)
    except Exception as e:
        print("[ERROR] 좌표계 파싱 실패:", e)
        return

    if executed_once:
        print("[Callback] 이미 한 번 실행됨 → 무시")
        return

    # ❗ 여기서는 "동작해도 된다" 플래그만 세우고, 실제 로봇 제어는 main 루프에서 수행
    move_requested = True
    print("[Callback] move_requested = True 설정 (실제 이동은 main 루프에서 실행)")


# =========================================
#        한 번만 모션 실행하는 래퍼
# =========================================
def run_motion_once():
    global executed_once, move_requested, received_coord

    executed_once = True
    move_requested = False

    move_using_coord(received_coord)

    print("[Main] 이동 완료")


# =========================================
#          FORCE PUBLISH FUNCTION
# =========================================
def publish_force(d_fx, d_fy, d_fz):
    global force_pub
    msg = String()
    msg.data = f"{d_fx:.3f},{d_fy:.3f},{d_fz:.3f}"
    force_pub.publish(msg)


# new_coord pub
def publish_new_coord(coord):
    global new_coord
    msg = String()
    msg.data = f"{coord}"
    new_coord.publish(msg)


# panel pub
def publish_panel(theta, height, width):
    global panel_pub
    msg = String()
    msg.data = f"{theta:.3f},{height:.3f},{width:.3f}"
    panel_pub.publish(msg)

# =========================================
#        좌표계 기반 이동 (실제 로봇 제어)
# =========================================
def move_using_coord(received_coord):
    from DSR_ROBOT2 import (
        posx, movel, set_ref_coord, set_user_cart_coord,
        DR_BASE, DR_MV_MOD_REL, wait, get_current_posx
    )

    print("\n=== 좌표계 등록 ===")
    print(received_coord)

    x, y, z, rx, ry, rz = received_coord
    coord_posx = posx(x, y, z, rx, ry, rz)


    wait(1)

    print("[Node] set_user_cart_coord 호출")
    coor_id = set_user_cart_coord(coord_posx, ref=DR_BASE)
    print(f"[Node] 등록된 좌표계 ID = {coor_id}")

    set_ref_coord(coor_id)
    print("[Node] REF 좌표계 활성화 완료")

    home = posx(0,0,-1,33.174,-2.54,146.489)
    movel(home, v=50, a=50)
    # now = get_current_posx()[0]   # posx 리스트 (길이 6)
    # x, y, z, rx, ry, rz = now

    # home = posx(x, y, z, 0, 0, 0)
    # movel(home, v=50, a=50)

    lu = move_stop_lu(coor_id)
    rd = move_stop_rd(coor_id)

    theta = 180- ry
    height, width = width_calculation(lu, rd)

    publish_panel(theta,height,width)


    new_origin = [lu[0], lu[1], lu[2], rx, ry, rz]
    publish_new_coord(new_origin)
    print("[Node] new_coord Publish 완료 =", new_origin)
    
    # 변경 전
    # tool_position = posx([294.02, -301.17, 280.19-228.6, 40.53, -179.75, 40.87])
    # tool_approach = posx([294.02, -301.17, 449-228.6, 40.53, -179.75, 40.87])

        # 변경 후
    tool_position = posx([258.57, -294.36, 282.14-228.6, 4.38, -174.93, 91.16])
    tool_approach = posx([258.57, -294.36, 449-228.6, 4.38, -174.93, 91.16])

    movel(tool_approach, vel=VELOCITY, acc=ACC)
    movel(tool_position, vel=VELOCITY, acc=ACC)
    gripper_close()
    movel(tool_approach, vel=VELOCITY, acc=ACC)


# =========================================
#              FORCE BASED LU SCAN
# =========================================
def move_stop_lu(coor_id):
    from DSR_ROBOT2 import posx, movel, DR_MV_MOD_REL
    from DSR_ROBOT2 import get_tool_force, get_current_posx, wait, DR_TOOL

    wait(1)
    n = 5
    movel(posx(0-175, -120,-2,0,0,0), v=200, a=100, mod=DR_MV_MOD_REL)
    
    wait(0.5)

    # ↑ 위 탐색
    f0 = [0, 0, 0]
    f1 = [0, 0, 0]
    f2 = [0, 0, 0]
    f = [0, 0, 0]
    for i in range(n):
        movel(posx(0, -2, 0, 0, 0, 0), v=200, a=100, mod=DR_MV_MOD_REL)
        fx, fy, fz = get_tool_force(DR_TOOL)[0:3]
        f_avg = [f0[0] + fx, f0[1] + fy, f0[2] + fz]
    f_avg = [f0[0]/n, f0[1]/n, f0[2]/n]

    i = 0

    while True:
        fx, fy, fz = get_tool_force(DR_TOOL)[0:3]
        pos, sol = get_current_posx(ref=coor_id)
        x, y, z = pos[0], pos[1], pos[2]

        publish_force(fx - f[0], fy - f[1], fz - f[2])

        if fy - f[1] > 0.4 or fy - f_avg[1] > 0.6:
            print("접촉 (위쪽)")
            break
        movel(posx(0, -2, 0, 0, 0, 0), v=200, a=100, mod=DR_MV_MOD_REL)
        f2 = f1
        f1 = f0
        f0 = [fx, fy, fz]
        f = [(f0[0]+f1[0]+f2[0])/3, (f0[1]+f1[1]+f2[1])/3, (f0[2]+f1[2]+f2[2])/3]  
        i +=1        

    
    wait(1)
    movel(posx(0, 6, 0, 0, 0, 0), v=200, a=100, mod=DR_MV_MOD_REL)
    wait(1)


    f0 = [0, 0, 0]
    f1 = [0, 0, 0]
    f2 = [0, 0, 0]
    f = [0, 0, 0]

    for i in range(n):
        movel(posx(-2, 0, 0, 0, 0, 0), v=200, a=100, mod=DR_MV_MOD_REL)
        fx, fy, fz = get_tool_force(DR_TOOL)[0:3]
        f_avg = [f0[0] + fx, f0[1] + fy, f0[2] + fz]
    f_avg = [f0[0]/n, f0[1]/n, f0[2]/n]


    while True:
        fx, fy, fz = get_tool_force(DR_TOOL)[0:3]
        pos, sol = get_current_posx(ref=coor_id)
        x, y, z = pos[0], pos[1], pos[2]
        publish_force(fx - f[0], fy - f[1], fz - f[2])

        if fx - f[0] > 0.3 or fx - f_avg[0] > 0.6:
            print("접촉 (왼쪽)")
            break
        movel(posx(-2, 0, 0, 0, 0, 0), v=200, a=100, mod=DR_MV_MOD_REL)
        f2 = f1
        f1 = f0
        f0 = [fx, fy, fz]
        f = [(f0[0]+f1[0]+f2[0])/3, (f0[1]+f1[1]+f2[1])/3, (f0[2]+f1[2]+f2[2])/3]  
        i +=1

    wait(1)
    movel(posx(6, 0, 0, 0, 0, 0), v=200, a=100, mod=DR_MV_MOD_REL)
    wait(1)

    return get_current_posx(coor_id)[0]


# =========================================
#              FORCE BASED RD SCAN
# =========================================
def move_stop_rd(coor_id):
    from DSR_ROBOT2 import posx, movel, DR_MV_MOD_REL
    from DSR_ROBOT2 import get_tool_force, get_current_posx, wait, DR_TOOL
    n = 5

    f0 = [0, 0, 0]
    f1 = [0, 0, 0]
    f2 = [0, 0, 0]
    f = [0, 0, 0]

    home = posx(0,0,-1,5.619,-3.723,172.321)
    movel(home, v=50, a=50)

    wait(1)

    movel(posx(175,120,0,0,0,0), v=200, a=100, mod=DR_MV_MOD_REL)
    # ↓ 아래 탐색

    for i in range(n):
        movel(posx(0, 2, 0, 0, 0, 0), v=200, a=100, mod=DR_MV_MOD_REL)
        fx, fy, fz = get_tool_force(DR_TOOL)[0:3]
        f_avg = [f0[0] + fx, f0[1] + fy, f0[2] + fz]
    f_avg = [f0[0]/n, f0[1]/n, f0[2]/n]

    while True:
        fx, fy, fz = get_tool_force(DR_TOOL)[0:3]
        pos, sol = get_current_posx(ref=coor_id)
        # pos는 [x, y, z, rx, ry, rz] 형태의 리스트
        pos, sol = get_current_posx(ref=coor_id)
        x, y, z = pos[0], pos[1], pos[2]
        publish_force(fx - f[0], fy - f[1], fz - f[2])

        if fy - f[1] < -1.2 or fy - f_avg[1] < -1.5:
            print("접촉 (아래)")
            break
        movel(posx(0, 2, 0, 0, 0, 0), v=200, a=100, mod=DR_MV_MOD_REL)
        f2 = f1
        f1 = f0
        f0 = [fx, fy, fz]
        f = [(f0[0]+f1[0]+f2[0])/3, (f0[1]+f1[1]+f2[1])/3, (f0[2]+f1[2]+f2[2])/3]  
    wait(1)
    movel(posx(0, -6, 0, 0, 0, 0), v=200, a=100, mod=DR_MV_MOD_REL)
    wait(1)



    # → 오른쪽 탐색

    f0 = [0, 0, 0]
    f1 = [0, 0, 0]
    f2 = [0, 0, 0]
    f = [0, 0, 0]


    for i in range(n):
        movel(posx(2, 0, 0, 0, 0, 0), v=200, a=100, mod=DR_MV_MOD_REL)
        fx, fy, fz = get_tool_force(DR_TOOL)[0:3]
        f_avg = [f0[0] + fx, f0[1] + fy, f0[2] + fz]
    f_avg = [f0[0]/n, f0[1]/n, f0[2]/n]

    while True:
        fx, fy, fz = get_tool_force(DR_TOOL)[0:3]
        pos, sol = get_current_posx(ref=coor_id)
        x, y, z = pos[0], pos[1], pos[2]
        publish_force(fx - f[0], fy - f[1], fz - f[2])

        if fx - f[0] < -0.4 or fx - f_avg[0] < -1.0:
            print("접촉 (오른쪽)")
            break
        movel(posx(2, 0, 0, 0, 0, 0), v=200, a=100, mod=DR_MV_MOD_REL)
        f2 = f1
        f1 = f0
        f0 = [fx, fy, fz]
        f = [(f0[0]+f1[0]+f2[0])/3, (f0[1]+f1[1]+f2[1])/3, (f0[2]+f1[2]+f2[2])/3]  

    wait(1)
    movel(posx(-4, 0, 0, 0, 0, 0), v=200, a=100, mod=DR_MV_MOD_REL)
    wait(1)

    return get_current_posx()[0]

def width_calculation(lu, rd):
    from DSR_ROBOT2 import posx, movel, DR_MV_MOD_REL
    from DSR_ROBOT2 import get_tool_force, get_current_posx, wait, DR_TOOL
    
    height = lu[1]-rd[1]
    width = rd[0]-lu[0]

    print(f"높이: {height}  폭: {width}")

    return height, width

def gripper_close():
    from DSR_ROBOT2 import set_digital_output, wait, ON, OFF
    set_digital_output(1, ON)
    set_digital_output(2, OFF)
    wait(0.5)


if __name__ == "__main__":
    main()
