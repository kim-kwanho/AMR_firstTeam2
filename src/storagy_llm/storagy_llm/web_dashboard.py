#!/usr/bin/env python3
"""Web dashboard bridge node.

Serves the static frontend, streams the robot pose / nav goal over SSE,
and relays chat messages to the existing `llm_agent` ROS service.
The rest of the system (simulation, wander BT, agent service) is untouched.
"""
import json
import math
import socket
import threading
import time
import yaml
from pathlib import Path

import rclpy
import tf2_ros
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

import cv2
from cv_bridge import CvBridge

from action_msgs.msg import GoalStatus, GoalStatusArray
from nav_msgs.msg import Path as NavPath
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Int32, String
from storagy_interfaces.srv import Agent
from ament_index_python.packages import get_package_share_directory

from flask import Flask, Response, jsonify, request, send_from_directory

WEB_DIR = Path(get_package_share_directory('storagy_llm')) / 'web'
HTTP_PORT = 8091  # Keep 8090 free for storagy_program_ws on this machine
CHAT_TIMEOUT_SEC = 90.0

SERVICE_DOWN_MSG = (
    "LLM 에이전트 서비스(agent_service)가 아직 실행되지 않았습니다. "
    "터미널에서 `ros2 run storagy_llm agent_service`를 먼저 실행해 주세요."
)
CHAT_TIMEOUT_MSG = "응답 시간이 초과되었습니다. 잠시 후 다시 시도해 주세요."


def yaw_from_quaternion(qx, qy, qz, qw):
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


class WebDashboard(Node):
    def __init__(self):
        super().__init__('web_dashboard')
        default_map = str(Path(get_package_share_directory('storagy')) / 'map' / '1206_sim_1.yaml')
        self.declare_parameter('map_file', default_map)
        self.map_file = Path(
            self.get_parameter('map_file').get_parameter_value().string_value)
        self.declare_parameter('map_offset_x', 0.0)
        self.declare_parameter('map_offset_y', 0.0)
        self.map_offset_x = self.get_parameter('map_offset_x').get_parameter_value().double_value
        self.map_offset_y = self.get_parameter('map_offset_y').get_parameter_value().double_value
        self.map_info = self._load_map_info(self.map_file)
        self.map_png = self._load_map_png(self.map_info['image_path'])

        self.lock = threading.Lock()
        self.pose = None          # (x, y, yaw) in map frame
        self.goal = None          # (x, y) in map frame
        self.navigating = False
        self.wander_enabled = False
        self.events = []          # ring buffer of {'id', 't', 'text'}
        self.event_seq = 0
        self.goal_statuses = {}   # goal uuid bytes -> last seen status
        self.status_initialized = False

        self.bridge = CvBridge()
        self.camera_jpeg = None   # latest annotated frame, JPEG bytes
        self.camera_stamp = 0.0
        self.person_count = 0
        self.person_stamp = 0.0

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.plan_sub = self.create_subscription(
            NavPath, '/plan', self.plan_callback, 10)

        # Nav2 action status topic uses transient-local reliable QoS
        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.status_sub = self.create_subscription(
            GoalStatusArray, '/navigate_to_pose/_action/status',
            self.status_callback, status_qos)

        self.event_sub = self.create_subscription(
            String, '/robot_events', self.event_callback, 10)

        wander_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.wander_sub = self.create_subscription(
            Bool, '/wander_enabled', self.wander_callback, wander_qos)

        camera_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.camera_sub = self.create_subscription(
            Image, '/yolo/detected_image', self.camera_callback, camera_qos)

        self.person_sub = self.create_subscription(
            Int32, '/yolo/person_count', self.person_callback, 10)

        self.agent_cli = self.create_client(Agent, 'llm_agent')
        self.pose_timer = self.create_timer(0.1, self.update_pose)
        
        # R1 integration: subscribe to hide team state
        self.hide_state = 'FREEZE'
        self.hide_state_sub = self.create_subscription(
            String, '/hide/state', self.hide_state_callback, 10)
            
        self.get_logger().info("Web dashboard node initialized.")

    def _load_map_info(self, map_file: Path) -> dict:
        with open(map_file, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)
        image_path = Path(data['image'])
        if not image_path.is_absolute():
            image_path = map_file.parent / image_path
        image = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
        if image is None:
            raise RuntimeError(f"map image not found or unreadable: {image_path}")
        height, width = image.shape[:2]
        origin = data.get('origin', [0.0, 0.0, 0.0])
        return {
            'map_file': str(map_file),
            'image_path': str(image_path),
            'resolution': float(data['resolution']),
            'origin': [float(origin[0]), float(origin[1]), float(origin[2])],
            'width': int(width),
            'height': int(height),
        }

    def _load_map_png(self, image_path: str) -> bytes:
        image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        if image is None:
            raise RuntimeError(f"map image not found or unreadable: {image_path}")
        ok, buf = cv2.imencode('.png', image)
        if not ok:
            raise RuntimeError(f"failed to encode map image: {image_path}")
        return buf.tobytes()

    def add_event(self, text: str):
        with self.lock:
            self.event_seq += 1
            self.events.append({
                'id': self.event_seq,
                't': time.time(),
                'text': text,
            })
            del self.events[:-100]

    def event_callback(self, msg: String):
        self.add_event(msg.data)

    def hide_state_callback(self, msg: String):
        new_state = msg.data
        log_change = False
        with self.lock:
            if self.hide_state != new_state:
                log_change = True
            self.hide_state = new_state
        if log_change:
            self.add_event(f"숨는팀 상태 변경: {new_state}")

    def wander_callback(self, msg: Bool):
        with self.lock:
            self.wander_enabled = msg.data

    def person_callback(self, msg: Int32):
        self.person_count = msg.data
        self.person_stamp = time.time()

    def camera_callback(self, msg: Image):
        now = time.time()
        if now - self.camera_stamp < 0.066:  # cap encoding at ~15 fps
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ok:
                self.camera_jpeg = buf.tobytes()
                self.camera_stamp = now
        except Exception as e:
            self.get_logger().warn(f"camera frame conversion failed: {e}",
                                   throttle_duration_sec=5.0)

    def update_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', Time())
            q = tf.transform.rotation
            yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
            with self.lock:
                self.pose = (tf.transform.translation.x + self.get_parameter("map_offset_x").value,
                             tf.transform.translation.y + self.get_parameter("map_offset_y").value, yaw)
        except Exception:
            with self.lock:
                self.pose = None

    def plan_callback(self, msg: NavPath):
        if not msg.poses:
            return
        last = msg.poses[-1].pose.position
        gx = last.x + self.get_parameter("map_offset_x").value
        gy = last.y + self.get_parameter("map_offset_y").value
        with self.lock:
            is_new_goal = self.goal is None
            self.goal = (gx, gy)
        if is_new_goal:
            self.add_event(f"이동 목표 설정 (x={gx:.2f}, y={gy:.2f})")

    EVENT_BY_STATUS = {
        GoalStatus.STATUS_SUCCEEDED: "목표 지점 도착 ✅",
        GoalStatus.STATUS_CANCELED: "이동 취소됨",
        GoalStatus.STATUS_ABORTED: "이동 실패 (경로 중단)",
    }

    def status_callback(self, msg: GoalStatusArray):
        new_events = []
        for s in msg.status_list:
            gid = bytes(s.goal_info.goal_id.uuid)
            prev = self.goal_statuses.get(gid)
            if prev == s.status:
                continue
            self.goal_statuses[gid] = s.status
            # Skip the first (latched) array: it replays past goals
            if not self.status_initialized:
                continue
            if s.status in self.EVENT_BY_STATUS:
                new_events.append(self.EVENT_BY_STATUS[s.status])
        self.status_initialized = True
        if len(self.goal_statuses) > 200:
            self.goal_statuses.clear()

        active = any(
            s.status in (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING)
            for s in msg.status_list
        )
        with self.lock:
            self.navigating = active
            if not active:
                self.goal = None
        for text in new_events:
            self.add_event(text)

    def snapshot(self) -> dict:
        with self.lock:
            pose, goal, navigating = self.pose, self.goal, self.navigating
            wander, event_seq, hide_state = self.wander_enabled, self.event_seq, self.hide_state
        return {
            'pose': {'x': pose[0], 'y': pose[1], 'yaw': pose[2]} if pose else None,
            'goal': {'x': goal[0], 'y': goal[1]} if goal and navigating else None,
            'navigating': navigating,
            'wander': wander,
            'camera': (time.time() - self.camera_stamp) < 2.0,
            'people': (self.person_count
                       if (time.time() - self.person_stamp) < 2.0 else None),
            'event_seq': event_seq,
            'hide_state': hide_state,
        }

    def events_since(self, last_seq: int) -> list:
        with self.lock:
            return [e for e in self.events if e['id'] > last_seq]

    def ask(self, question: str) -> str:
        if not self.agent_cli.service_is_ready():
            return SERVICE_DOWN_MSG
        req = Agent.Request()
        req.question = question
        done = threading.Event()
        future = self.agent_cli.call_async(req)
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(CHAT_TIMEOUT_SEC):
            future.cancel()
            return CHAT_TIMEOUT_MSG
        if future.exception() is not None:
            self.get_logger().error(f"llm_agent call failed: {future.exception()}")
            return "LLM 서비스 호출 중 오류가 발생했습니다."
        return future.result().answer


def create_app(node: WebDashboard) -> Flask:
    app = Flask(__name__, static_folder=None)

    @app.route('/')
    def index():
        return send_from_directory(WEB_DIR, 'index.html')

    @app.route('/<path:fname>')
    def static_files(fname):
        return send_from_directory(WEB_DIR, fname)

    @app.route('/api/stream')
    def stream():
        def gen():
            last_seq = 0
            while True:
                payload = node.snapshot()
                if payload['event_seq'] > last_seq:
                    payload['events'] = node.events_since(last_seq)
                    last_seq = payload['event_seq']
                yield f"data: {json.dumps(payload)}\n\n"
                time.sleep(0.1)
        return Response(gen(), mimetype='text/event-stream',
                        headers={'Cache-Control': 'no-cache'})

    @app.route('/api/map-info')
    def map_info():
        info = node.map_info
        return jsonify({
            'resolution': info['resolution'],
            'origin': info['origin'],
            'width': info['width'],
            'height': info['height'],
        })

    @app.route('/api/map-image')
    def map_image():
        return Response(node.map_png, mimetype='image/png')

    @app.route('/api/camera')
    def camera():
        def gen():
            last_stamp = 0.0
            while True:
                if node.camera_jpeg is not None and node.camera_stamp != last_stamp:
                    last_stamp = node.camera_stamp
                    frame = node.camera_jpeg
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n'
                           b'Content-Length: ' + str(len(frame)).encode() + b'\r\n\r\n'
                           + frame + b'\r\n')
                time.sleep(0.05)
        return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')

    @app.route('/api/chat', methods=['POST'])
    def chat():
        body = request.get_json(silent=True) or {}
        question = str(body.get('question', '')).strip()
        if not question:
            return jsonify({'answer': '질문이 비어 있습니다.'}), 400
        return jsonify({'answer': node.ask(question)})

    return app


def _http_port_in_use(port: int) -> bool:
    """대시보드 포트가 이미 열려 있으면 중복 web_dashboard 실행으로 간주."""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.settimeout(0.3)
        return sock.connect_ex(('127.0.0.1', port)) == 0


def main(args=None):
    if _http_port_in_use(HTTP_PORT):
        print(
            f'[web_dashboard] 포트 {HTTP_PORT}이(가) 이미 사용 중입니다. '
            '중복 실행을 막기 위해 종료합니다. '
            '기존 프로세스를 끈 뒤 다시 실행하세요: '
            'pkill -f "storagy_llm/web_dashboard"',
            flush=True,
        )
        raise SystemExit(1)

    rclpy.init(args=args)
    node = WebDashboard()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    app = create_app(node)
    node.get_logger().info(f"Dashboard ready: http://localhost:{HTTP_PORT}")
    try:
        app.run(host='0.0.0.0', port=HTTP_PORT, threaded=True, use_reloader=False)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
