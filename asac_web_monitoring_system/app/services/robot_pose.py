import threading
import rclpy
from rclpy.node import Node
from dsr_msgs2.srv import GetCurrentPosx


class RobotPoseNode(Node):
    def __init__(self):
        super().__init__("robot_pose_node")

        self.cli_posx = self.create_client(
            GetCurrentPosx,
            "/dsr01/aux_control/get_current_posx"
        )

        while not self.cli_posx.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for get_current_posx service...")

        self._last_pose = None

        # request at 20ms
        self.create_timer(0.02, self.request_pose)

    def request_pose(self):
        req = GetCurrentPosx.Request()
        req.ref = 0

        future = self.cli_posx.call_async(req)
        future.add_done_callback(self._pose_response_cb)

    def _pose_response_cb(self, future):
        try:
            resp = future.result()
        except Exception:
            return

        if resp is None or not resp.success:
            return

        if not resp.task_pos_info:
            return

        arr = resp.task_pos_info[0]
        data = arr.data

        if len(data) < 3:
            return

        self._last_pose = {
            "x": float(data[0]),
            "y": float(data[1]),
            "z": float(data[2]),
        }

    def get_current_pose(self):
        return self._last_pose


_pose_node = None
_rclpy_init = False


def init_ros_once():
    global _pose_node, _rclpy_init

    if _rclpy_init:
        return

    rclpy.init()
    _pose_node = RobotPoseNode()

    th = threading.Thread(target=lambda: rclpy.spin(_pose_node), daemon=True)
    th.start()

    _rclpy_init = True


def get_robot_pose():
    global _pose_node

    if not _rclpy_init:
        init_ros_once()

    if _pose_node is None:
        return None

    return _pose_node.get_current_pose()
