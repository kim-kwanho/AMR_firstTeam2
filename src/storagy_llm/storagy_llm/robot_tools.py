import math
import time
import rclpy
import tf2_ros
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool, String
from storagy_interfaces.srv import Emotion
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue
from langchain_core.tools import StructuredTool
from typing import List

def create_tools(tool_set) -> List[StructuredTool]:
    tool_specs = [
        {
            "name": "get_current_location",
            "description": "사용자가 현재 로봇의 위치나 근처 장소를 물어봤을 때 사용합니다. 각 장소와 거리를 계산해 가장 가까운 위치를 반환합니다.",
            "func": lambda: tool_set.get_current_location()
        },
        {
            "name": "list_locations",
            "description": "사용자가 어디로 이동할 수 있는지 물어봤을 때 사용합니다. 이동 가능한 장소 목록을 보여줍니다.",
            "func": lambda: ", ".join(tool_set.list_locations())
        },
        {
            "name": "move_to_location",
            "description": "사용자가 특정 장소로 이동하라고 말했을 때 사용합니다.",
            "func": lambda place: tool_set.move_to_location(place)
        },
        {
            "name": "start_guide",
            "description": "사용자가 준비 완료되었거나 출발(가자)하라고 말했을 때 안내 주행을 시작합니다.",
            "func": lambda: tool_set.start_guide()
        },
        {
            "name": "explain_front_camera",
            "description": "사용자가 앞에 무엇이 보이는지, 혹은 카메라 이미지를 분석하여 현재 로봇 전방의 상황을 설명해 달라고 했을 때 사용합니다.",
            "func": lambda: tool_set.explain_front_camera()
        },
        {
            "name": "cancel_navigation",
            "description": "사용자가 이동을 취소하거나, 로봇을 멈추라고(정지) 명령했을 때 사용합니다. 랜덤 이동과 네비게이션을 모두 중단하고 로봇을 정지시킵니다.",
            "func": lambda: tool_set.cancel_navigation()
        },
        {
            "name": "start_random_wander",
            "description": "사용자가 랜덤 이동(자유 주행, 배회, 돌아다니기)을 시작하라고 했을 때 사용합니다.",
            "func": lambda: tool_set.set_wander(True)
        },
        {
            "name": "stop_random_wander",
            "description": "사용자가 랜덤 이동(배회)을 멈추라고 했을 때 사용합니다. 진행 중인 랜덤 이동 목표도 즉시 취소됩니다.",
            "func": lambda: tool_set.set_wander(False)
        },
        {
            "name": "move_robot",
            "description": "지정한 거리만큼 로봇을 직진 이동시킵니다 (텔레옵). direction은 'forward'(앞) 또는 'backward'(뒤), distance_m은 미터 단위 거리. 예: '앞으로 1미터 가줘' → direction='forward', distance_m=1.0. 실행 전 랜덤 이동/네비게이션은 자동으로 정지됩니다.",
            "func": lambda direction, distance_m: tool_set.teleop_move(direction, distance_m)
        },
        {
            "name": "rotate_robot",
            "description": "로봇을 제자리에서 회전시킵니다 (텔레옵). direction은 'left' 또는 'right', angle_deg는 회전 각도(도). 예: '오른쪽으로 돌아' → right, 90 / '뒤로 돌아(반바퀴)' → left, 180. 실행 전 랜덤 이동/네비게이션은 자동으로 정지됩니다.",
            "func": lambda direction, angle_deg: tool_set.teleop_rotate(direction, angle_deg)
        },
        {
            "name": "set_speed",
            "description": "텔레옵 속도를 설정하거나 조회합니다. linear_mps=직진 속도(m/s, 0.05~0.7), angular_rps=회전 속도(rad/s, 0.2~1.8). 바꾸지 않을 값은 0을 전달하세요 (둘 다 0이면 현재 속도 조회). '더 빠르게/느리게' 요청 시 먼저 조회한 뒤 적절히 가감해 다시 호출하세요.",
            "func": lambda linear_mps, angular_rps: tool_set.set_speed(linear_mps, angular_rps)
        },
        {
            "name": "complete_mission",
            "description": "사용자가 메뉴판을 정상적으로 수령했거나, 서비스 서빙 완료를 보고하여 로봇의 임무를 종료하고 복귀를 지시할 때 사용합니다.",
            "func": lambda: tool_set.complete_mission()
        },
        {
            "name": "get_hide_state",
            "description": "숨는팀 FSM 현재 상태(FREEZE/GUIDE/DOCK 등)를 조회합니다. '지금 상태', '은폐 중이야?' 등에 사용합니다.",
            "func": lambda: tool_set.get_hide_state()
        },
        {
            "name": "start_takeover",
            "description": "사용자가 '기상', '임무 교대', '나와', '손님 왔어' 등 기상/교대를 요청할 때 사용합니다. FREEZE에서 GUIDE로 전환합니다.",
            "func": lambda: tool_set.start_takeover()
        },
        {
            "name": "finish_mission",
            "description": "안내 임무가 끝났을 때('미션 끝', '서비스 완료', '은폐처로 돌아가') 복귀를 시작합니다.",
            "func": lambda: tool_set.finish_mission()
        },
        {
            "name": "finish_docking",
            "description": "은폐처 도킹이 완료됐을 때('도킹 완료', '은폐 완료') FREEZE 상태로 돌아갑니다.",
            "func": lambda: tool_set.finish_docking()
        },
    ]

    tools = [
        StructuredTool.from_function(
            func=spec["func"],
            name=spec["name"],
            description=spec["description"]
        )
        for spec in tool_specs
    ]

    return tools

class ToolSet(Node):
    def __init__(self, places: dict, explain_fn=None):
        super().__init__('tool_set')
        self.places = places
        self.explain_fn = explain_fn
        self.frame_id = "map"
        self.base_frame = "base_link"
        self.tolerance_m = 0.3

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=True)
        self.ac = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.llm_active_pub = self.create_publisher(Bool, '/llm_active', 10)

        # Teleop / wander control state
        self.linear_speed = 0.2    # m/s
        self.angular_speed = 0.8   # rad/s
        self.wander_enabled = False
        wander_qos = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            depth=1
        )
        self.wander_pub = self.create_publisher(Bool, '/wander_enabled', wander_qos)
        self.event_pub = self.create_publisher(String, '/robot_events', 10)

        # Hide FSM (P1) — LLM → /hide/* topic pub
        self.hide_state = 'UNKNOWN'
        self.hide_takeover_pub = self.create_publisher(Bool, '/hide/takeover_start', 10)
        self.hide_mission_done_pub = self.create_publisher(Bool, '/hide/mission_done', 10)
        self.hide_dock_done_pub = self.create_publisher(Bool, '/hide/dock_done', 10)
        self.create_subscription(
            String, '/hide/state', self._on_hide_state, wander_qos)

        self.goal_handle = None
        self.pending_place = None  # move_to_location → start_guide 2단계 이동
        # R1 / R4 integration with hide team
        self.takeover_pub = self.create_publisher(Bool, '/hide/takeover_start', 10)
        self.mission_done_pub = self.create_publisher(Bool, '/hide/mission_done', 10)
        self.emotion_cli = self.create_client(Emotion, '/hide/set_emotion')
        self.param_client = self.create_client(SetParameters, '/guide_nav_node/set_parameters')
        self.person_arrived_pub = self.create_publisher(Bool, '/guide/person_arrived', 10)
        self.guide_command_pub = self.create_publisher(String, '/guide/command', 10)

    def publish_llm_active(self, active: bool):
        msg = Bool()
        msg.data = active
        self.llm_active_pub.publish(msg)

    def publish_event(self, text: str):
        msg = String()
        msg.data = text
        self.event_pub.publish(msg)

    def _on_hide_state(self, msg: String):
        self.hide_state = msg.data

    def _require_guide(self) -> str | None:
        if self.hide_state == 'FREEZE':
            return ("[HIDE] FREEZE(위장) 상태라 이동할 수 없습니다. "
                    "먼저 start_takeover로 기상/임무 교대를 해 주세요.")
        return None

    def get_hide_state(self) -> str:
        return f"[HIDE] 현재 상태: {self.hide_state}"

    def start_takeover(self) -> str:
        self.hide_takeover_pub.publish(Bool(data=True))
        self.publish_event("임무 교대(기상) 신호 발행")
        return "[HIDE] takeover_start 발행 — FREEZE→GUIDE 전이 요청"

    def finish_mission(self) -> str:
        self.hide_mission_done_pub.publish(Bool(data=True))
        self.publish_event("미션 완료 신호 발행")
        return "[HIDE] mission_done 발행 — 복귀(DOCK) 전이 요청"

    def finish_docking(self) -> str:
        self.hide_dock_done_pub.publish(Bool(data=True))
        self.publish_event("도킹 완료 신호 발행")
        return "[HIDE] dock_done 발행 — FREEZE 전이 요청"

    def set_wander(self, enabled: bool) -> str:
        if enabled:
            blocked = self._require_guide()
            if blocked:
                return blocked
        self.wander_enabled = enabled
        msg = Bool()
        msg.data = enabled
        self.wander_pub.publish(msg)
        if enabled:
            self.publish_event("랜덤 이동 시작")
            return "[WANDER] 랜덤 이동을 시작했습니다."
        self.publish_event("랜덤 이동 정지")
        return "[WANDER] 랜덤 이동을 정지했습니다."

    def _stop_all_motion(self):
        """Stop wandering and any active navigation so teleop has exclusive cmd_vel."""
        if self.wander_enabled:
            self.set_wander(False)
        if self.goal_handle is not None:
            try:
                self.goal_handle.cancel_goal_async()
            except Exception as e:
                self.get_logger().warn(f"[TELEOP] Failed to cancel LLM goal: {e}")
            self.goal_handle = None
            self.publish_llm_active(False)
        # Notify guide_nav_node to stop/pause
        msg = String()
        msg.data = "stop"
        self.guide_command_pub.publish(msg)
        self.cmd_pub.publish(Twist())
        time.sleep(0.5)  # give Nav2 a moment to release cmd_vel

    @staticmethod
    def _normalize_angle(a: float) -> float:
        return (a + math.pi) % (2.0 * math.pi) - math.pi

    def _teleop_pose(self):
        """(x, y, yaw) for teleop feedback. Prefers odom (no AMCL jumps), falls back to map."""
        for frame in ('odom', self.frame_id):
            try:
                tf = self.tf_buffer.lookup_transform(frame, self.base_frame, Time(),
                                                     timeout=Duration(seconds=0.2))
                q = tf.transform.rotation
                yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                return (tf.transform.translation.x, tf.transform.translation.y, yaw)
            except Exception:
                continue
        return None

    def teleop_move(self, direction, distance_m) -> str:
        blocked = self._require_guide()
        if blocked:
            return blocked
        try:
            distance = abs(float(distance_m))
        except (TypeError, ValueError):
            return "[ERR] 이동 거리 값을 이해하지 못했습니다."
        if distance <= 0.0 or distance > 5.0:
            return "[ERR] 이동 거리는 0보다 크고 5m 이하만 가능합니다."

        backward = str(direction).lower().startswith('b')
        self._stop_all_motion()

        start = self._teleop_pose()
        if start is None:
            return "[ERR] 로봇 위치(TF)를 확인할 수 없어 이동할 수 없습니다."
        sx, sy = start[0], start[1]
        sign = -1.0 if backward else 1.0
        speed = self.linear_speed

        deadline = time.time() + distance / speed * 3.0 + 5.0
        traveled = 0.0
        tw = Twist()
        while time.time() < deadline:
            cur = self._teleop_pose()
            if cur is not None:
                traveled = math.hypot(cur[0] - sx, cur[1] - sy)
                if traveled >= distance:
                    break
            # Slow down near the target for a clean stop
            remaining = distance - traveled
            v = speed if remaining > 0.15 else max(0.05, speed * 0.4)
            tw.linear.x = sign * v
            self.cmd_pub.publish(tw)
            time.sleep(0.05)
        self.cmd_pub.publish(Twist())
        time.sleep(0.5)  # let the velocity smoother finish decelerating
        cur = self._teleop_pose()
        if cur is not None:
            traveled = math.hypot(cur[0] - sx, cur[1] - sy)

        label = "뒤로" if backward else "앞으로"
        self.publish_event(f"텔레옵: {label} {traveled:.2f}m 이동 (목표 {distance:.2f}m)")
        if traveled < distance * 0.9:
            return (f"[TELEOP] {label} 이동이 {traveled:.2f}m에서 멈췄습니다 "
                    f"(목표 {distance:.2f}m). 장애물이나 시간 초과일 수 있습니다.")
        return f"[TELEOP] {label} {traveled:.2f}m 이동을 완료했습니다."

    def teleop_rotate(self, direction, angle_deg) -> str:
        blocked = self._require_guide()
        if blocked:
            return blocked
        try:
            angle = abs(float(angle_deg))
        except (TypeError, ValueError):
            angle = 90.0
        if angle <= 0.0 or angle > 360.0:
            return "[ERR] 회전 각도는 0보다 크고 360도 이하만 가능합니다."

        right = str(direction).lower().startswith('r')
        self._stop_all_motion()

        start = self._teleop_pose()
        if start is None:
            return "[ERR] 로봇 위치(TF)를 확인할 수 없어 회전할 수 없습니다."
        target = math.radians(angle)
        sign = -1.0 if right else 1.0
        speed = self.angular_speed

        deadline = time.time() + target / speed * 3.0 + 5.0
        prev_yaw = start[2]
        turned = 0.0
        tw = Twist()
        while time.time() < deadline:
            cur = self._teleop_pose()
            if cur is not None:
                turned += abs(self._normalize_angle(cur[2] - prev_yaw))
                prev_yaw = cur[2]
                if turned >= target:
                    break
            # Slow down near the target for a clean stop
            remaining = target - turned
            w = speed if remaining > math.radians(12.0) else max(0.2, speed * 0.4)
            tw.angular.z = sign * w
            self.cmd_pub.publish(tw)
            time.sleep(0.05)
        self.cmd_pub.publish(Twist())
        time.sleep(0.5)  # let the velocity smoother finish decelerating
        cur = self._teleop_pose()
        if cur is not None:
            turned += abs(self._normalize_angle(cur[2] - prev_yaw))

        turned_deg = math.degrees(turned)
        label = "오른쪽" if right else "왼쪽"
        self.publish_event(f"텔레옵: {label}으로 {turned_deg:.0f}° 회전 (목표 {angle:.0f}°)")
        if turned_deg < angle * 0.9:
            return (f"[TELEOP] {label} 회전이 {turned_deg:.0f}도에서 멈췄습니다 "
                    f"(목표 {angle:.0f}도). 시간 초과일 수 있습니다.")
        return f"[TELEOP] {label}으로 {turned_deg:.0f}도 회전을 완료했습니다."

    def set_speed(self, linear_mps=0, angular_rps=0) -> str:
        try:
            linear = float(linear_mps)
            angular = float(angular_rps)
        except (TypeError, ValueError):
            return "[ERR] 속도 값을 이해하지 못했습니다."

        changed = False
        if linear > 0:
            self.linear_speed = min(max(linear, 0.05), 0.7)
            changed = True
        if angular > 0:
            self.angular_speed = min(max(angular, 0.2), 1.8)
            changed = True

        status = (f"직진 {self.linear_speed:.2f}m/s, "
                  f"회전 {self.angular_speed:.2f}rad/s")
        if changed:
            self.publish_event(f"속도 변경: {status}")
            return f"[SPEED] 속도를 설정했습니다. 현재 {status}"
        return f"[SPEED] 현재 속도: {status}"

    def lookup_current_pose(self):
        try:
            # We look up transform with a timeout to prevent locking if TF is not fully available yet
            tf = self.tf_buffer.lookup_transform(self.frame_id, self.base_frame, Time(), timeout=Duration(seconds=1.0))
            return (tf.transform.translation.x,
                    tf.transform.translation.y,
                    tf.transform.rotation)
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return None

    def get_current_location(self) -> str:
        cur = self.lookup_current_pose()
        if not cur:
            return "[ERR] 위치 확인 실패 (TF lookup failed)"
        cx, cy, _ = cur

        best, dist = None, 1e9
        for name, (x, y, qz, qw) in self.places.items():
            d = math.hypot(cx - x, cy - y)
            if d < dist:
                best, dist = name, d

        if dist < self.tolerance_m:
            return f"[HERE] {best}"
        else:
            return f"[HERE] Unknown (nearest={best}, dist={dist:.2f}m)"

    def list_locations(self):
        return list(self.places.keys())

    def explain_front_camera(self) -> str:
        if self.explain_fn:
            return self.explain_fn()
        return "[ERR] 이미지 분석 및 설명 기능을 사용할 수 없습니다."

    def navigate_to_pose(self, x, y, qz=0.0, qw=1.0, done_callback=None):
        self.publish_llm_active(True)
        # WAKE trigger to wake up the hiding robot
        wake_msg = Bool()
        wake_msg.data = True
        self.takeover_pub.publish(wake_msg)
        self.publish_event("안내 시작: 숨는팀 WAKE 트리거 전송")
        goal = NavigateToPose.Goal()
        ps = PoseStamped()
        ps.header.frame_id = self.frame_id
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.orientation.z = float(qz)
        ps.pose.orientation.w = float(qw)
        goal.pose = ps

        self.get_logger().info("[NAV] Waiting for navigation action server...")
        self.ac.wait_for_server()
        self.get_logger().info(f"[NAV] Sending navigation goal: x={x:.2f}, y={y:.2f}")

        self._goal_future = self.ac.send_goal_async(goal)

        def goal_response_callback(future):
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().warn("[NAV] Goal rejected")
                self.goal_handle = None
                self.publish_llm_active(False)
                if done_callback:
                    done_callback(False)
            else:
                self.get_logger().info("[NAV] Goal accepted")
                self.goal_handle = goal_handle
                self._result_future = goal_handle.get_result_async()
                self._result_future.add_done_callback(result_callback)

        def result_callback(future):
            self.goal_handle = None
            self.publish_llm_active(False)
            result = future.result()
            status = result.status
            if status == 4: # STATUS_SUCCEEDED
                self.get_logger().info("[NAV] Goal reached successfully!")
                self.get_logger().info("[AUDIO] 목적지에 도착하였습니다. 메뉴판을 수령해주세요.")
                self.publish_event("도착 완료! 메뉴판 수령을 대기합니다.")
                if self.emotion_cli.service_is_ready():
                    req = Emotion.Request()
                    req.emotion = "happy"
                    self.emotion_cli.call_async(req)
                if done_callback:
                    done_callback(True)
            else:
                self.get_logger().warn(f"[NAV] Navigation failed with status: {status}")
                if done_callback:
                    done_callback(False)

        self._goal_future.add_done_callback(goal_response_callback)

    def start_guide(self) -> str:
        blocked = self._require_guide()
        if blocked:
            return blocked
        if not self.pending_place:
            return ("[NAV] 이동할 목적지가 설정되지 않았습니다. "
                    "먼저 장소를 말씀해 주세요 (예: T2로 가줘).")

        place = self.pending_place
        x, y, qz, qw = self.places[place]
        self.pending_place = None
        self.publish_event(f"출발 지시 — {place}(x={x:.2f}, y={y:.2f})로 Nav2 주행 시작")
        self.navigate_to_pose(x, y, qz, qw)
        return f"[GUIDE] {place}(으)로 안내 주행을 시작합니다."

    def move_to_location(self, place: str):
        blocked = self._require_guide()
        if blocked:
            return blocked
        if place not in self.places:
            return f"[ERR] Unknown place: {place}"

        self.pending_place = place
        x, y, qz, qw = self.places[place]
        # Calculate yaw from quaternion (qz, qw)
        yaw = 2.0 * math.atan2(qz, qw)

        # guide_nav_node가 떠 있으면 파라미터도 동기화 (full_bringup 미포함 시 무시)
        if not self.param_client.service_is_ready():
            self.get_logger().debug('guide_nav_node not running; pending_place only')
        req = SetParameters.Request()
        
        px = Parameter()
        px.name = 'target_x'
        px.value.type = 3 # ParameterType.PARAMETER_DOUBLE
        px.value.double_value = float(x)
        
        py = Parameter()
        py.name = 'target_y'
        py.value.type = 3 # ParameterType.PARAMETER_DOUBLE
        py.value.double_value = float(y)
        
        pyaw = Parameter()
        pyaw.name = 'target_yaw'
        pyaw.value.type = 3 # ParameterType.PARAMETER_DOUBLE
        pyaw.value.double_value = float(yaw)
        
        req.parameters = [px, py, pyaw]
        
        self.publish_event(
            f"'{place}' (x={x:.2f}, y={y:.2f}, yaw={yaw:.2f})로 목적지 설정 — 출발 대기")
        if self.param_client.service_is_ready():
            self.param_client.call_async(req)
        
        return f"[NAV] 목적지가 {place}(으)로 설정되었습니다. 준비 완료되면 출발하라고 말씀해주세요."

    def cancel_navigation(self) -> str:
        self.pending_place = None
        stopped = []
        if self.goal_handle is not None:
            try:
                self.goal_handle.cancel_goal_async()
            except Exception as e:
                self.get_logger().warn(f"[NAV] Failed to cancel goal: {e}")
            self.goal_handle = None
            self.publish_llm_active(False)
            stopped.append("Nav2 주행")
        if self.wander_enabled:
            self.set_wander(False)
            stopped.append("랜덤 이동")
        
        # Notify guide_nav_node to stop/pause
        msg = String()
        msg.data = "stop"
        self.guide_command_pub.publish(msg)
        stopped.append("안내 주행")
        
        # Publish zero velocity to stop the robot
        stop_msg = Twist()
        self.cmd_pub.publish(stop_msg)
        self.publish_event("정지 명령: " + ", ".join(stopped) + " 중지")
        return f"[NAV] {', '.join(stopped)}을(를) 중지하고 로봇을 정지시켰습니다."

    def complete_mission(self) -> str:
        # 1. Stop all navigation/wandering to yield control
        self._stop_all_motion()
        
        # 2. Publish mission_done to return control to hide team
        msg = Bool()
        msg.data = True
        self.mission_done_pub.publish(msg)
        
        # 3. Reset emotion to basic
        if self.emotion_cli.service_is_ready():
            req = Emotion.Request()
            req.emotion = "basic"
            self.emotion_cli.call_async(req)
            
        self.publish_event("안내 서비스 종료. 제어권 숨는팀 이양")
        return "[MISSION] 서비스 완료 처리를 마쳤습니다. 로봇이 복귀 모드로 전환됩니다."
