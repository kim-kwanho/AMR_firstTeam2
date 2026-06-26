#!/usr/bin/env python3
"""P3(파트 A) — 사람 감지 + 120도 keepout 부채꼴 산출.

토이 가이드(컨셉 3): 매장 손님(인간)의 "시야각(120°)"을 진입금지 영역으로 만들어,
로봇이 사람 시선/동선을 피해 구석 사각지대로 복귀·은폐하는 근거로 쓴다(백그라운드 상시).

파이프라인:
  /camera/color/image_raw --YOLO(person)--> 박스 중심(u,v)
  /camera/depth/image_raw --> 중심 깊이 z
  (u,v,z) + camera_info K --> 카메라 기준 사람 평면 좌표(x 전방, y 좌)
  사람마다 전방 fov_deg 부채꼴 폴리곤 --> /hide/keepout_zones

토픽 계약:
  - 구독: /camera/color/image_raw, /camera/color/camera_info, /camera/depth/image_raw
  - 발행: /hide/persons(PoseArray)            # 사람 상대 위치(카메라 좌표)
          /hide/keepout_zones(PolygonStamped)  # 사람 전방 120도 부채꼴
          /hide/human_detected(Bool)           # FSM/대시보드용 간단 신호
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool
from geometry_msgs.msg import Point32, Pose, PoseArray, PolygonStamped
from cv_bridge import CvBridge
from ultralytics import YOLO


class HumanPerception(Node):
    def __init__(self):
        super().__init__('hide_human_perception')

        self.declare_parameter('fov_deg', 120.0)
        self.declare_parameter('zone_radius_m', 1.0)
        self.declare_parameter('yolo_conf', 0.5)
        self.declare_parameter('model', 'yolov8n.pt')
        self.declare_parameter('min_depth_m', 0.2)
        self.declare_parameter('max_depth_m', 8.0)
        self.declare_parameter('frame_id', 'camera_link')

        self.fov_deg = self.get_parameter('fov_deg').value
        self.zone_radius = self.get_parameter('zone_radius_m').value
        self.yolo_conf = self.get_parameter('yolo_conf').value
        self.min_depth = self.get_parameter('min_depth_m').value
        self.max_depth = self.get_parameter('max_depth_m').value
        self.frame_id = self.get_parameter('frame_id').value

        model_name = self.get_parameter('model').value
        self.get_logger().info(f'YOLO 로드: {model_name}')
        self.model = YOLO(model_name)
        self.bridge = CvBridge()

        # 카메라 토픽은 RELIABLE 로 발행되므로 구독도 RELIABLE 로 맞춘다.
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.persons_pub = self.create_publisher(PoseArray, '/hide/persons', 10)
        self.zones_pub = self.create_publisher(PolygonStamped, '/hide/keepout_zones', 10)
        self.detected_pub = self.create_publisher(Bool, '/hide/human_detected', 10)

        self.K = None
        self.latest_depth = None

        self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self._on_info, image_qos)
        self.create_subscription(
            Image, '/camera/color/image_raw', self._on_color, image_qos)
        self.create_subscription(
            Image, '/camera/depth/image_raw', self._on_depth, image_qos)

        self.get_logger().info('P3 HumanPerception 시작')

    def _on_info(self, msg: CameraInfo):
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)

    def _on_depth(self, msg: Image):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'depth 변환 실패: {e}', throttle_duration_sec=5.0)
            return
        depth = np.asarray(depth, dtype=np.float32)
        # 16UC1(mm) 인 경우 m 로 환산. 32FC1 은 이미 m.
        if msg.encoding in ('16UC1', 'mono16'):
            depth = depth / 1000.0
        self.latest_depth = depth

    def _on_color(self, msg: Image):
        if self.K is None or self.latest_depth is None:
            return

        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'color 변환 실패: {e}', throttle_duration_sec=5.0)
            return

        results = self.model(img, classes=[0], conf=self.yolo_conf, verbose=False)

        persons = PoseArray()
        persons.header.stamp = msg.header.stamp
        persons.header.frame_id = self.frame_id

        depth = self.latest_depth
        dh, dw = depth.shape[:2]
        fx, fy = self.K[0, 0], self.K[1, 1]
        cx, cy = self.K[0, 2], self.K[1, 2]

        for box in results[0].boxes.xyxy.cpu().numpy():
            u = int((box[0] + box[2]) / 2.0)
            v = int((box[1] + box[3]) / 2.0)
            u = max(0, min(dw - 1, u))
            v = max(0, min(dh - 1, v))

            z = float(depth[v, u])
            if not math.isfinite(z) or not (self.min_depth < z < self.max_depth):
                continue

            cam_x = (u - cx) * z / fx  # 광학축 기준 우측(+)
            pose = Pose()
            pose.position.x = z        # 전방
            pose.position.y = -cam_x   # 좌(+)
            pose.position.z = 0.0
            pose.orientation.w = 1.0
            persons.poses.append(pose)

        # 최근접 우선 정렬(전방 거리 기준)
        persons.poses.sort(key=lambda p: p.position.x)

        self.persons_pub.publish(persons)
        self.detected_pub.publish(Bool(data=bool(persons.poses)))
        self._publish_zones(persons)

        if persons.poses:
            self.get_logger().info(
                f'사람 {len(persons.poses)}명: 최근접 {persons.poses[0].position.x:.2f}m',
                throttle_duration_sec=2.0)

    def _publish_zones(self, persons: PoseArray):
        """각 사람 위치에서 로봇 방향으로 fov_deg 부채꼴 진입금지 영역 생성."""
        half = math.radians(self.fov_deg / 2.0)
        steps = 8
        for p in persons.poses:
            poly = PolygonStamped()
            poly.header = persons.header
            px, py = p.position.x, p.position.y
            # 사람→로봇(카메라 원점) 방향을 부채꼴 중심축으로 사용
            base = math.atan2(-py, -px)
            poly.polygon.points.append(Point32(x=float(px), y=float(py), z=0.0))
            for k in range(steps + 1):
                a = base - half + (2.0 * half) * k / steps
                poly.polygon.points.append(Point32(
                    x=float(px + self.zone_radius * math.cos(a)),
                    y=float(py + self.zone_radius * math.sin(a)),
                    z=0.0))
            self.zones_pub.publish(poly)


def main(args=None):
    rclpy.init(args=args)
    node = HumanPerception()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
