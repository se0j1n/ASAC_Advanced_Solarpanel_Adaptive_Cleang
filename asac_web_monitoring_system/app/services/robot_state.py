import rclpy
from rclpy.node import Node
from fastapi import APIRouter
import json
from std_msgs.msg import Int32, Float32, String


# FastAPI가 공유하는 로봇 상태 저장 딕셔너리
robot_state = {
    "ui_state": "🟢 대기 중",
    "robot_state": "IDLE",
    "selected_tool": 0,
    "overall_progress": 0,
    "current_action": "None",
    "measured_angle": None
}

# 서버 최초 1번만 초기화하도록 하는 플래그
progress_reset_done = False

robot_node = None



# ROS2 Subscriber + Publisher
class RobotStateSubscriber(Node):
    def __init__(self):
        super().__init__("system_state_subscriber")

        # /system_state JSON 수신
        self.create_subscription(String, "/system_state", self.system_state_cb, 10)

        # 측정된 각도 수신
        self.create_subscription(String, "/dsr01/calculated_coord", self.angle_cb, 10)

        # Node1 status 수신
        self.create_subscription(String, "/robot_status", self.status_cb, 10)

        # 퍼블리셔
        self.tool_select_pub = self.create_publisher(Int32, "/tool_select", 10)
        self.angle_pub = self.create_publisher(Float32, "/panel_angle", 10)
        self.start_signal_pub = self.create_publisher(String, "/dsr01/start_signal", 10)

        self.prev_progress = -1
        self.get_logger().info("✅ RobotStateSubscriber Ready")

    def system_state_cb(self, msg):
        global progress_reset_done

        try:
            data = json.loads(msg.data)
        except:
            self.get_logger().error("❌ JSON 파싱 실패")
            return

        robot_state["robot_state"] = data.get("robot_state", "IDLE")
        robot_state["selected_tool"] = data.get("selected_tool", 0)
        robot_state["overall_progress"] = data.get("overall_progress", 0)
        robot_state["current_action"] = data.get("current_action", "None")

        progress = robot_state["overall_progress"]
        rs = robot_state["robot_state"]
        tool = robot_state["selected_tool"]

        # 진행률 변화 로그 (필요할 때만)
        if progress != self.prev_progress:
            print(f"[ROS] 진행률 {progress:.2f}%")
            self.prev_progress = progress


        #   서버 최초 1번만 초기화
        if not progress_reset_done:
            if rs == "IDLE" and tool == 0:
                robot_state["overall_progress"] = 0
                progress_reset_done = True
                print("[WEB] 서버 최초 실행: 진행률 초기화 완료")


        # UI 표시용 상태 설정
        if rs == "IDLE":
            robot_state["ui_state"] = "🔵 작업 완료" if progress >= 100 else "🟢 대기 중"

        elif rs == "TOOL_GRIP_FRONT":
            robot_state["ui_state"] = "🟠 걸레 준비 중" if tool == 1 else "🟠 물 분사 준비 중"

        # elif rs == "WAITING_ANGLE":
        #     robot_state["ui_state"] = "패널overall_progress 각도 측정 중"

        elif rs == "WORKING":
            robot_state["ui_state"] = "🔴 걸레 청소 중" if tool == 1 else "🔴 물 분사 중"

        elif rs == "TOOL_GRIP_BACK":
            robot_state["ui_state"] = "🟠 도구 정리 중"

        else: 
            robot_state["ui_state"] = "⚪상태 인식 불가"


    # 측정된 각도 수신 
    def angle_cb(self, msg):
        theta = float(msg.data)
        robot_state["measured_angle"] = theta
        print(f"[ROS] 📐 측정된 각도 수신 = {theta:.2f}°")


    # Node1 상태 수신 
    def status_cb(self, msg):
        print(f"[ROS] STATUS : {msg.data}")


# FastAPI와 연결할 ROS Node 생성 함수

def create_robot_state_node():
    global robot_node
    rclpy.init()
    robot_node = RobotStateSubscriber()
    return robot_node

router = APIRouter(prefix="/api")


@router.get("/work_state")
async def get_work_state():
    if robot_state["ui_state"] == "🟢 대기 중":
        robot_state["overall_progress"] = 0
    return robot_state

# 버튼 1: 각도 측정 시작
@router.post("/start_angle_measure")
async def start_angle_measure():
    global robot_node
    if robot_node is None:
        return {"error": "robot_node not ready"}

    msg = String()
    msg.data = "START"
    robot_node.start_signal_pub.publish(msg)
    return {"status": "angle measuring started"}


# 버튼 2: 모드 선택
@router.post("/select_tool")
async def select_tool(payload: dict):
    global robot_node

    tool = payload.get("tool")
    if tool not in [1, 2]:
        return {"error": "invalid tool"}

    msg = Int32()
    msg.data = tool
    robot_node.tool_select_pub.publish(msg)

    robot_state["selected_tool"] = tool

    return {"status": "tool selected", "tool": tool}


# 버튼 3: 작업 시작 → 저장된 θ 값 전송
@router.post("/start_cleaning")
async def start_cleaning():
        # 새 작업 시작 → progress 초기화
    robot_state["overall_progress"] = 0
    robot_state["ui_state"] = "🟢 작업 시작 준비 중"

    if robot_state["measured_angle"] is None:
        return {"error": "no measured angle"}

    msg = Float32()
    msg.data = float(robot_state["measured_angle"])
    robot_node.angle_pub.publish(msg)

    return {
        "status": "cleaning_started",
        "angle": robot_state["measured_angle"],
        "tool": robot_state["selected_tool"]
    }
