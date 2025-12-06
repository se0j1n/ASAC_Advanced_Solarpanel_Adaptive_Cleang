#!/usr/bin/env python3
import rclpy
import math
import DR_init
import time
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from dsr_rokey2.action import WipePanel


# ==========================
# 로봇 설정 변수
# ==========================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA"

VELOCITY = 60
ACC = 60

# ==========================
# 패널 좌표 설정
# ==========================
PANEL_TOP_LEFT = [488.97, 176.43, 400.83, 2.78, 140.57, 95.83]
PANEL_TOP_RIGHT = [488.97, -210.58, 400.83, 2.78, 140.55, 95.83]

panel_height = 300
BASE_ANGLE = 43.8       # 기준 패널 각도
BASE_RY = 140.55        # 기준 Ry 값

y_left = PANEL_TOP_LEFT[1]
y_right = PANEL_TOP_RIGHT[1]
y_center = (y_left + y_right) / 2
panel_width_y = abs(y_left - y_right)
half_width_y = panel_width_y / 2

tool_width = 150
plus_clean = 20
step_width = tool_width - plus_clean

# 몇 번 반복할지 계산
n = int(panel_height / step_width) + 1
h = panel_height / (n - 1) if n > 1 else panel_height

# ==========================
# DR_init 설정
# ==========================
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


# ==========================
# Action Server 정의
# ==========================
class WipeFloorServer(Node):
    def __init__(self):
        super().__init__("wipe_floor_server", namespace=ROBOT_ID)

        # DR_init에 현재 노드 전달
        DR_init.__dsr__node = self

        # Action Server 생성
        self._action_server = ActionServer(
            self,
            WipePanel,
            "wipe_floor_action",
            self.execute_callback
        )

        # 로봇 초기화
        self.initialize_robot()

        self.get_logger().info("### Wipe Floor Action Server Ready ###")

    # ------------------------------------------
    # 로봇 초기화 (Tool / TCP 설정)
    # ------------------------------------------
    def initialize_robot(self):
        from DSR_ROBOT2 import set_tool, set_tcp
        set_tool(ROBOT_TOOL)
        set_tcp(ROBOT_TCP)
        self.get_logger().info("### Robot Initialized (WipeFloor) ###")

    # ------------------------------------------
    # 패널 각도 → Ry 변환
    # ------------------------------------------
    def calculate_ry(self, panel_angle):
        angle_diff = panel_angle - BASE_ANGLE
        return BASE_RY - angle_diff

    # ------------------------------------------
    # 시작 자세 계산
    # ------------------------------------------
    def get_start_position(self, panel_angle):
        ry = self.calculate_ry(panel_angle)
        return [478.97, y_center, 380.83, 2.78, ry, 95.83]

    # ------------------------------------------
    # Action 실행 콜백
    # ------------------------------------------
    def execute_callback(self, goal_handle):
        from DSR_ROBOT2 import (
            movel, posx, wait,
            DR_BASE, DR_MV_MOD_REL
        )

        # Goal에서 angle 가져오기
        panel_angle = goal_handle.request.start_angle
        self.get_logger().info(f">>> Wipe Floor Action Received (angle={panel_angle:.2f}°)")

        feedback = WipePanel.Feedback()
        result = WipePanel.Result()

        # 시작 위치로 이동
        start_pos = self.get_start_position(panel_angle)
        self.get_logger().info("🚩 시작 위치로 이동")
        movel(posx(start_pos), vel=VELOCITY, acc=ACC)
        wait(0.5)

        self.get_logger().info("🧽 패널 닦기 수행 시작...")
        total_passes = n
        self.get_logger().info(f"🔄 총 {total_passes}번 반복 진행")

        for i in range(n):
            # Progress 퍼센트 계산
            progress = float((i / total_passes) * 100)
            feedback.progress = progress
            goal_handle.publish_feedback(feedback)

            self.get_logger().info(f"[PASS {i+1}/{n}] panel_angle={panel_angle:.2f}°")

            # CENTER → LEFT
            movel(posx(0, half_width_y, 0, 0, 0, 0),
                  vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL, ref=DR_BASE)
            wait(0.2)

            # LEFT → RIGHT
            movel(posx(0, -panel_width_y, 0, 0, 0, 0),
                  vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL, ref=DR_BASE)
            wait(0.2)

            # RIGHT → CENTER
            movel(posx(0, half_width_y, 0, 0, 0, 0),
                  vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL, ref=DR_BASE)
            wait(0.2)

            # 아래로 내려가기
            if i < n - 1:
                cos_val = math.cos(math.radians(panel_angle))
                sin_val = math.sin(math.radians(panel_angle))
                dx = -(h * cos_val) / 1.7
                dz = -(h * sin_val) / 1.7

                movel(
                    posx(dx, 0, dz, 0, 0, 0),
                    vel=VELOCITY, acc=ACC, mod=DR_MV_MOD_REL, ref=DR_BASE
                )
                wait(0.3)

        # 100% 완료
        feedback.progress = 100.0
        goal_handle.publish_feedback(feedback)
        result.success = True
        result.message = "Wipe panel completed successfully"
        goal_handle.succeed()

        self.get_logger().info("✅ Wipe Floor Completed Successfully!")
        self.get_logger().info("🔧 다음 작업 준비 완료\n")

        return result


# ==========================
# main()
# ==========================
def main(args=None):
    rclpy.init(args=args)

    # DR_init 초기화를 위한 임시 노드 생성
    temp_node = rclpy.create_node('temp_wipe_floor_init', namespace=ROBOT_ID)
    DR_init.__dsr__node = temp_node

    # 실제 ActionServer 노드 생성
    node = WipeFloorServer()

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
