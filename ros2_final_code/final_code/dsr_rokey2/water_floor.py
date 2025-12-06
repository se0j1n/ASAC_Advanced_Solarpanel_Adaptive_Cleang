#!/usr/bin/env python3
import rclpy
import DR_init
import time
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Float32
from dsr_rokey2.action import WaterPatrol


# ==========================
# 로봇 설정 상수
# ==========================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v1"

VELOCITY = 100
ACC = 50
BASE_PANEL_ANGLE = 43.8

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# Ready 자세
JReady = [1.01, -8.64, 96.48, -0.93, 95.54, 89.08]

# (좌상단 → 우상단 → 우하단 → 좌하단)
BASE_COORDINATES = [
    [489.39, 185.72, 420.24, 1.00, 168.21, 90.79],       # 좌상단
    [489.39, -197.37, 420.24, 1.00, 168.21, 90.79],      # 우상단
    [418.54, -197.37, 296.45, 175.34, 165.29, -94.44],   # 우하단
    [418.54, 185.72, 296.45, 175.34, 165.29, -94.44],    # 좌하단
]


# ==========================
# 패널 각도 보정 함수
# ==========================
def apply_angle_offset(base_coord, angle_now):
    """패널 현재 각도(angle_now)에 따라 좌표 및 Ry를 보정"""
    angle_diff = angle_now - BASE_PANEL_ANGLE

    x, y, z, Rx, Ry, Rz = base_coord
    corrected_Ry = Ry - angle_diff

    # x(전후), z(상하) 보정
    if angle_diff < 0:
        x_new = x - angle_diff * 4
        z_new = z
    else:
        x_new = x - angle_diff * 4
        z_new = z + angle_diff * 0.3

    return [x_new, y, z_new, Rx, corrected_Ry, Rz]


# ==========================
# Action Server 클래스
# ==========================
class WaterFloorServer(Node):
    def __init__(self):
        super().__init__("water_floor_server", namespace=ROBOT_ID)

        # DR_init에 현재 노드 등록
        DR_init.__dsr__node = self

        # ActionServer 생성
        self._action_server = ActionServer(
            self,
            WaterPatrol,
            "water_floor_action",
            self.execute_callback
        )

        # 로봇 초기 설정
        self.initialize_robot()

        self.get_logger().info("### Water Floor Action Server Ready ###")

    # ------------------------------------------
    # 로봇 초기화 (Tool / TCP 설정)
    # ------------------------------------------
    def initialize_robot(self):
        from DSR_ROBOT2 import set_tool, set_tcp
        set_tool(ROBOT_TOOL)
        set_tcp(ROBOT_TCP)
        self.get_logger().info("### Robot Initialized (WaterFloor) ###")

    # ------------------------------------------
    # Action 실행 콜백
    # ------------------------------------------
    def execute_callback(self, goal_handle):
        from DSR_ROBOT2 import posx, movej, movel

        angle_now = goal_handle.request.angle
        self.get_logger().info(f">>> WaterFloor Action Received (angle={angle_now:.2f}°)")

        feedback_msg = WaterPatrol.Feedback()
        result_msg = WaterPatrol.Result()

        # Ready 자세 이동
        self.get_logger().info("🚩 Ready 자세로 이동 중...")
        movej(JReady, vel=VELOCITY, acc=ACC)

        # 각도 보정된 경로 생성
        corrected_positions = [
            posx(apply_angle_offset(coord, angle_now))
            for coord in BASE_COORDINATES
        ]

        total_points = len(corrected_positions)
        self.get_logger().info(f"🔍 총 {total_points}개 지점 순회 시작...")

        # 포인트 순회
        for i, pos in enumerate(corrected_positions, 1):
            self.get_logger().info(f"📍 지점 {i}/{total_points} 이동 중...")
            movel(pos, vel=VELOCITY, acc=ACC)

            feedback_msg.progress = float((i / total_points) * 100)
            goal_handle.publish_feedback(feedback_msg)

            time.sleep(0.1)

        # 완료 처리
        self.get_logger().info("✅ 물 터누리 청소 완료!")
        result_msg.success = True
        result_msg.message = "Water patrol completed successfully"
        goal_handle.succeed()

        self.get_logger().info("🔧 다음 작업 준비 완료\n")
        return result_msg


# ==========================
# main()
# ==========================
def main(args=None):
    rclpy.init(args=args)

    # 임시 초기화 노드 생성 (DR_init 등록용)
    temp_node = rclpy.create_node("temp_water_floor_init", namespace=ROBOT_ID)
    DR_init.__dsr__node = temp_node

    # 실제 ActionServer 노드 생성
    node = WaterFloorServer()

    executor = MultiThreadedExecutor()
    executor.add_node(temp_node)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        temp_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
