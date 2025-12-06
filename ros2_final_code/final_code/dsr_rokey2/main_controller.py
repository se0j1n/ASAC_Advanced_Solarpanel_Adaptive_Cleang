#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import Int32, Float32, String
import json

from dsr_rokey2.action import WipePanel, WaterPatrol
from dsr_msgs2.srv import SetRobotControl, SetSafetyMode, GetRobotState  # ⭐ 추가

ROBOT_ID = "dsr01"

# Doosan 상태 코드 → 문자열 (디버그용)
DSR_STATE_NAMES = {
    0: "INITIALIZING",
    1: "STANDBY",
    2: "MOVING",
    3: "SAFE_OFF",
    4: "TEACHING",
    5: "SAFE_STOP",
    6: "EMERGENCY_STOP",
    7: "HOMING",
    8: "RECOVERY",
    9: "SAFE_STOP2",
    10: "SAFE_OFF2",
}
SAFE_OK_STATES = (1, 2, 4, 7)  # 작업 가능한 상태들


class MainController(Node):
    def __init__(self):
        super().__init__('main_controller', namespace=ROBOT_ID)

        # -----------------------------
        # 상태 변수
        # -----------------------------
        self.selected_tool = None
        self.panel_angle = None
        self.is_running_sequence = False
        self.waiting_for_angle = False

        self.current_action = "None"
        self.action_progress = 0.0
        self.overall_progress = 0.0
        self.robot_state = "IDLE"
        self.current_step = "STEP_0"
        self.has_error = False
        self.error_message = ""

        # ⭐ 큰 단계(Phase) 관리
        # TOOL_GRIP_FRONT / ACTION / TOOL_GRIP_BACK / NONE
        self.current_phase = "NONE"
        self.phase_retry = {
            "TOOL_GRIP_FRONT": 0,
            "ACTION": 0,
            "TOOL_GRIP_BACK": 0,
        }
        self.max_phase_retry = 5

        # -----------------------------
        # Action Clients
        # -----------------------------
        self.wipe_client = ActionClient(self, WipePanel, "/dsr01/wipe_floor_action")
        self.water_client = ActionClient(self, WaterPatrol, "/dsr01/water_floor_action")

        # -----------------------------
        # Subscribers
        # -----------------------------
        self.create_subscription(Int32, "/tool_select", self.tool_select_callback, 10)
        self.create_subscription(Int32, "/task_done", self.task_done_callback, 10)
        self.create_subscription(Float32, "/panel_angle", self.angle_callback, 10)

        # -----------------------------
        # Publishers
        # -----------------------------
        self.task_end_pub = self.create_publisher(Int32, "/task_end", 10)
        self.tool_grip_start_pub = self.create_publisher(Int32, "/tool_grip_start", 10)

        # JSON 상태 publisher
        self.state_pub = self.create_publisher(String, "/system_state", 10)

        # Coverage map 시작 신호 publisher
        self.start_cleaning_pub = self.create_publisher(String, "start_cleaning", 10)

        # 10Hz 타이머
        self.create_timer(0.1, self.publish_state)

        # -----------------------------
        # 시스템 시작 Log
        # -----------------------------
        self.get_logger().info("=== Main Controller Ready ===")
        self.get_logger().info("💡 새로운 작업을 시작하려면 /tool_select를 publish하세요\n")

    # -----------------------------------------------------------
    # JSON 상태 Publish
    # -----------------------------------------------------------
    def publish_state(self):
        """기존 SystemState msg 대신 JSON 문자열 publish"""

        data = {
            "robot_state": self.robot_state,
            "current_step": self.current_step,
            "selected_tool": self.selected_tool if self.selected_tool else 0,
            "tool_name": self.get_tool_name(self.selected_tool),
            "overall_progress": float(self.overall_progress),
            "action_progress": float(self.action_progress),
            "panel_angle": float(self.panel_angle) if self.panel_angle else 0.0,
            "is_running": self.is_running_sequence,
            "waiting_for_angle": self.waiting_for_angle,
            "current_action": self.current_action,
            "has_error": self.has_error,
            "error_message": self.error_message,
            "current_phase": self.current_phase,
            "phase_retry": self.phase_retry,
        }

        msg = String()
        msg.data = json.dumps(data, ensure_ascii=False)
        self.state_pub.publish(msg)

    # -----------------------------------------------------------
    # Helper
    # -----------------------------------------------------------
    def get_tool_name(self, tool_id):
        if tool_id == 1:
            return "걸레"
        elif tool_id == 2:
            return "물뿌리개"
        return "없음"

    def update_state(self, robot_state, current_step, overall_progress=None):
        self.robot_state = robot_state
        self.current_step = current_step
        if overall_progress is not None:
            self.overall_progress = overall_progress

    # ===========================================================
    # Doosan 상태 조회 + 자동 복구 관련
    # ===========================================================
    def get_dsr_state(self):
        """Doosan GetRobotState 서비스로 실제 상태 코드 읽기"""
        try:
            client = self.create_client(
                GetRobotState,
                f"/{ROBOT_ID}/system/get_robot_state"
            )
            if not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().error("[AUTO_RECOVERY] GetRobotState 서비스 없음")
                return None

            req = GetRobotState.Request()
            future = client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)

            if future.result() and future.result().success:
                state = future.result().robot_state
                self.get_logger().info(
                    f"[AUTO_RECOVERY][STATE] {DSR_STATE_NAMES.get(state, state)} ({state})"
                )
                return state

            self.get_logger().error("[AUTO_RECOVERY] 상태 조회 실패 (result 없음 또는 success=False)")
            return None

        except Exception as e:
            self.get_logger().error(f"[AUTO_RECOVERY] 상태 조회 오류: {e}")
            return None

    def reset_safe_stop(self, state):
        """SAFE_STOP(5, 9) 해제"""
        if state not in (5, 9):
            return True
        self.get_logger().warn("[AUTO_RECOVERY] SAFE_STOP → CONTROL_RESET_SAFE_STOP 시도")
        try:
            client = self.create_client(
                SetRobotControl,
                f"/{ROBOT_ID}/system/set_robot_control"
            )
            if not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().error("[AUTO_RECOVERY] SetRobotControl 없음(SAFE_STOP)")
                return False

            req = SetRobotControl.Request()
            req.robot_control = 2  # CONTROL_RESET_SAFE_STOP
            future = client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)

            if future.result() and future.result().success:
                self.get_logger().info("[AUTO_RECOVERY] SAFE_STOP 리셋 성공")
                return True
            self.get_logger().error("[AUTO_RECOVERY] SAFE_STOP 리셋 실패")
            return False
        except Exception as e:
            self.get_logger().error(f"[AUTO_RECOVERY] SAFE_STOP 해제 오류: {e}")
            return False

    def reset_safe_off(self, state):
        """SAFE_OFF(3, 10) 해제 (Servo ON)"""
        if state not in (3, 10):
            return True
        self.get_logger().warn("[AUTO_RECOVERY] SAFE_OFF → CONTROL_RESET_SAFE_OFF 시도")
        try:
            client = self.create_client(
                SetRobotControl,
                f"/{ROBOT_ID}/system/set_robot_control"
            )
            if not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().error("[AUTO_RECOVERY] SetRobotControl 없음(SAFE_OFF)")
                return False

            req = SetRobotControl.Request()
            req.robot_control = 3  # CONTROL_RESET_SAFE_OFF (Servo ON)
            future = client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)

            if future.result() and future.result().success:
                self.get_logger().info("[AUTO_RECOVERY] SAFE_OFF 해제 성공 (Servo ON)")
                return True
            self.get_logger().error("[AUTO_RECOVERY] SAFE_OFF 해제 실패")
            return False
        except Exception as e:
            self.get_logger().error(f"[AUTO_RECOVERY] SAFE_OFF 해제 오류: {e}")
            return False

    def recovery_complete(self):
        """RECOVERY COMPLETE 이벤트"""
        self.get_logger().warn("[AUTO_RECOVERY] RECOVERY COMPLETE 이벤트 시도")
        try:
            client = self.create_client(
                SetSafetyMode,
                f"/{ROBOT_ID}/system/set_safety_mode"
            )
            if not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().error("[AUTO_RECOVERY] SetSafetyMode 없음(COMPLETE)")
                return False

            req = SetSafetyMode.Request()
            req.safety_mode = 2   # SAFETY_MODE_RECOVERY
            req.safety_event = 2  # SAFETY_MODE_EVENT_COMPLETE

            future = client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)

            if future.result() and future.result().success:
                self.get_logger().info("[AUTO_RECOVERY] RECOVERY COMPLETE 성공")
                return True
            self.get_logger().error("[AUTO_RECOVERY] RECOVERY COMPLETE 실패")
            return False
        except Exception as e:
            self.get_logger().error(f"[AUTO_RECOVERY] RECOVERY COMPLETE 오류: {e}")
            return False

    def recovery_exit(self):
        """RECOVERY EXIT (CONTROL_RESET_RECOVERY)"""
        self.get_logger().warn("[AUTO_RECOVERY] RECOVERY EXIT 시도")
        try:
            client = self.create_client(
                SetRobotControl,
                f"/{ROBOT_ID}/system/set_robot_control"
            )
            if not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().error("[AUTO_RECOVERY] SetRobotControl 없음(RECOVERY EXIT)")
                return False

            req = SetRobotControl.Request()
            req.robot_control = 7  # CONTROL_RESET_RECOVERY

            future = client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)

            if future.result() and future.result().success:
                self.get_logger().info("[AUTO_RECOVERY] RECOVERY EXIT 성공")
                return True
            self.get_logger().error("[AUTO_RECOVERY] RECOVERY EXIT 실패")
            return False
        except Exception as e:
            self.get_logger().error(f"[AUTO_RECOVERY] RECOVERY EXIT 오류: {e}")
            return False

    def auto_recovery(self):
        """SAFE_* / RECOVERY → STANDBY/MOVING까지 자동 복구"""
        self.get_logger().info("===== [AUTO_RECOVERY] 자동 복구 절차 시작 =====")

        state = self.get_dsr_state()
        if state is None:
            self.get_logger().error("[AUTO_RECOVERY] 현재 상태를 알 수 없음 → 중단")
            return False

        # SAFE_STOP 처리
        if state in (5, 9):
            if not self.reset_safe_stop(state):
                return False
            state = self.get_dsr_state()
            if state is None:
                return False

        # SAFE_OFF 처리
        if state in (3, 10):
            if not self.reset_safe_off(state):
                return False
            state = self.get_dsr_state()
            if state is None:
                return False

        # RECOVERY 상태 처리
        if state == 8:
            self.recovery_complete()
            state = self.get_dsr_state()
            if state is None:
                return False

            if state == 8:
                self.recovery_exit()
                state = self.get_dsr_state()
                if state is None:
                    return False

        # 최종 상태 확인
        if state in SAFE_OK_STATES:
            self.get_logger().info(
                f"[AUTO_RECOVERY] 정상 상태로 복귀 완료 → {DSR_STATE_NAMES.get(state, state)}({state})"
            )
            return True

        self.get_logger().error(
            f"[AUTO_RECOVERY] 복구 후에도 정상 상태 아님 → {DSR_STATE_NAMES.get(state, state)}({state})"
        )
        return False

    def ensure_phase_ok_or_recover(self, phase_name: str, restart_fn):
        """
        어떤 큰 단계(phase)가 '끝났다'고 신호가 왔을 때 호출.
        - 로봇 상태가 SAFE_* / RECOVERY면: auto_recovery + 해당 phase 재시작(restart_fn)
        - 정상 상태면: True 리턴
        - 복구/재시작을 수행한 경우: False 리턴 (caller는 여기서 return 해야 함)
        """
        state = self.get_dsr_state()
        if state is None:
            self.get_logger().warn("[AUTO_RECOVERY] 상태 조회 실패 → 일단 통과")
            return True

        if state in SAFE_OK_STATES:
            return True

        # SAFE_* / RECOVERY 상태 → 복구 + phase 재시작
        retries = self.phase_retry.get(phase_name, 0)
        if retries >= self.max_phase_retry:
            self.get_logger().error(
                f"[AUTO_RECOVERY] {phase_name} 복구 재시도 초과 → ERROR 처리"
            )
            self.has_error = True
            self.error_message = f"{phase_name} 중 안전정지 후 복구 실패"
            self.update_state("ERROR", "ERROR", 0.0)
            return False

        self.phase_retry[phase_name] = retries + 1

        self.get_logger().warn(
            f"[AUTO_RECOVERY] {phase_name} 중 안전모드 감지: "
            f"{DSR_STATE_NAMES.get(state, state)}({state}) → 복구 + 단계 재시작 ({retries+1}/{self.max_phase_retry})"
        )

        if not self.auto_recovery():
            self.has_error = True
            self.error_message = f"{phase_name} 중 자동 복구 실패"
            self.update_state("ERROR", "ERROR", 0.0)
            return False

        # 복구 성공 → 해당 phase 다시 시작
        if restart_fn is not None:
            restart_fn()
        return False

    # -----------------------------------------------------------
    # STEP 0 → STEP 1
    # -----------------------------------------------------------
    def tool_select_callback(self, msg):
        if self.is_running_sequence:
            self.get_logger().warn("⚠️ 이미 작업 중입니다.")
            return

        self.selected_tool = msg.data
        tool_name = self.get_tool_name(msg.data)

        # Phase/Retry 초기화
        self.current_phase = "TOOL_GRIP_FRONT"
        for k in self.phase_retry.keys():
            self.phase_retry[k] = 0

        self.get_logger().info(f"\n{'='*50}")
        self.get_logger().info(f"[STEP 0] 새 작업 시작: {tool_name} 선택됨")
        self.get_logger().info(f"{'='*50}")

        self.update_state("TOOL_GRIP_FRONT", "STEP_0", 0.0)
        self.start_tool_grip_frontend()

    def start_tool_grip_frontend(self):
        msg = Int32()
        msg.data = self.selected_tool
        self.tool_grip_start_pub.publish(msg)

        self.get_logger().info("[STEP 1] 📤 tool_grip 전반부 시작 명령 발행")
        self.update_state("TOOL_GRIP_FRONT", "STEP_1", 10.0)

    # -----------------------------------------------------------
    # task_done 콜백
    # -----------------------------------------------------------
    def task_done_callback(self, msg):
        if msg.data == 1:
            # 툴 잡기 전반부 완료 신호
            self.current_phase = "TOOL_GRIP_FRONT"

            # 실제 로봇 상태 확인 → SAFE면 복구 + 전반부 전체 재시작
            if not self.ensure_phase_ok_or_recover(
                phase_name="TOOL_GRIP_FRONT",
                restart_fn=self.start_tool_grip_frontend,
            ):
                # 복구/재시작 수행 → 다음 단계로 넘어가면 안 됨
                return

            # 여기까지 왔으면 정상 상태에서 전반부가 성공적으로 끝난 것
            if not self.is_running_sequence:
                self.get_logger().info("[STEP 1] ✅ tool_grip 전반부 완료")
                self.get_logger().info("[STEP 2] ⏳ panel_angle 대기 중...")
                self.is_running_sequence = True
                self.waiting_for_angle = True
                self.update_state("WAITING_ANGLE", "STEP_2", 25.0)
            return

        if msg.data == 2:
            # 툴 놓기(후반부) 완료 신호
            self.current_phase = "TOOL_GRIP_BACK"

            # SAFE면 복구 + /task_end 다시 발행해서 후반부 전체 재시작
            if not self.ensure_phase_ok_or_recover(
                phase_name="TOOL_GRIP_BACK",
                restart_fn=self.publish_task_end,
            ):
                return

            self.get_logger().info("[STEP 4] ✅ tool_grip 후반부 완료")
            self.get_logger().info("="*50)
            self.get_logger().info("🏁 전체 작업 완료!")
            self.get_logger().info("💡 새로운 작업을 시작하려면 /tool_select를 publish하세요\n")

            self.reset_state()
            return

        if msg.data == -1:
            self.get_logger().error("❌ tool_grip 동작 실패")
            self.has_error = True
            self.error_message = "Tool grip failed"
            self.update_state("ERROR", "ERROR", 0.0)

    def reset_state(self):
        self.is_running_sequence = False
        self.waiting_for_angle = False
        self.panel_angle = None
        self.selected_tool = None
        self.current_action = "None"
        self.action_progress = 0.0
        self.current_phase = "NONE"
        for k in self.phase_retry.keys():
            self.phase_retry[k] = 0
        self.update_state("IDLE", "STEP_0", 100.0)

    # -----------------------------------------------------------
    # panel_angle 수신
    # -----------------------------------------------------------
    def angle_callback(self, msg):
        self.panel_angle = msg.data

        if self.waiting_for_angle and self.is_running_sequence:
            self.get_logger().info(f"[STEP 2] ✅ 패널 각도 수신: {msg.data:.2f}°")
            self.waiting_for_angle = False
            self.update_state("WORKING", "STEP_3", 40.0)

            # 액션 phase 시작
            self.current_phase = "ACTION"
            self.phase_retry["ACTION"] = 0

            self.execute_sequence()

    # -----------------------------------------------------------
    # 액션 실행
    # -----------------------------------------------------------
    def execute_sequence(self):
        tool = self.selected_tool
        angle = self.panel_angle

        if tool == 1:
            self.get_logger().info("[STEP 3] 🧹 WipePanel 실행 중...")
            self.current_action = "WipePanel"
            self.call_wipe_panel_async(angle)

        elif tool == 2:
            self.get_logger().info("[STEP 3] 💦 WaterPatrol 실행 중...")
            self.current_action = "WaterPatrol"
            self.call_water_patrol_async(angle)

    # --------------------------- Action 호출 ---------------------------
    def call_wipe_panel_async(self, angle):
        goal = WipePanel.Goal()
        goal.start_angle = angle

        # Coverage tracking 시작 신호 발행
        start_msg = String()
        start_msg.data = "START"
        self.start_cleaning_pub.publish(start_msg)
        self.get_logger().info("[COVERAGE] 📊 Coverage tracking 시작 신호 발행")

        self.wipe_client.wait_for_server()
        future = self.wipe_client.send_goal_async(
            goal,
            feedback_callback=self.action_feedback_callback
        )
        future.add_done_callback(self.goal_response_callback)

    def call_water_patrol_async(self, angle):
        goal = WaterPatrol.Goal()
        goal.angle = angle

        self.water_client.wait_for_server()
        future = self.water_client.send_goal_async(
            goal,
            feedback_callback=self.action_feedback_callback
        )
        future.add_done_callback(self.goal_response_callback)

    # 액션 피드백
    def action_feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.action_progress = fb.progress
        self.overall_progress = 40.0 + fb.progress * 0.4

    # 액션 결과 콜백
    def goal_response_callback(self, future):
        goal_handle = future.result()
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result

        if result.success:
            # ACTION phase 완료 시점
            self.current_phase = "ACTION"

            # 로봇 상태 확인 → SAFE면 복구 + execute_sequence() 다시 호출
            if not self.ensure_phase_ok_or_recover(
                phase_name="ACTION",
                restart_fn=self.execute_sequence,
            ):
                # 복구/재시작 수행 → STEP 4로 넘어가면 안 됨
                return

            self.get_logger().info(f"[STEP 3] ✅ 액션 완료: {result.message}")

            self.publish_task_end()
            self.get_logger().info("[STEP 4] ⏳ tool_grip 후반부 진행 중...")

            self.current_action = "None"
            self.action_progress = 0.0
            self.update_state("TOOL_GRIP_BACK", "STEP_4", 80.0)

        else:
            self.get_logger().error("❌ 액션 실패!")
            self.has_error = True
            self.error_message = "Action failed"
            self.update_state("ERROR", "ERROR", 0.0)

    # 후반부 작업 시작
    def publish_task_end(self):
        msg = Int32()
        msg.data = 1
        self.task_end_pub.publish(msg)
        self.get_logger().info("[STEP 4] 📤 /task_end 발행")


def main(args=None):
    rclpy.init(args=args)
    node = MainController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
