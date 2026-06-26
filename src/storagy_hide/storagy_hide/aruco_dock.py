#!/usr/bin/env python3
"""R2 — ArUco 정밀 도킹 + 복귀 주행.

담당(R2):
  - 월드 1206_2.sdf 구석에 aruco_0 대기석 배치, points.yaml 에 hideout 좌표 추가
  - /camera/color/image_raw + camera_info 로 cv2.aruco 태그 pose 추정
  - 정렬+접근 P제어로 /cmd_vel 발행, 거리 임계값 도달 시 /hide/dock_done(True) 발행
  - 복귀(RETURN): hideout 좌표로 Nav2 navigate_to_pose 후 도킹 시작
  - (HW) 엔코더 기반 정밀 정지로 교체 가능

상태 게이트: R1 FSM 이 RETURN/DOCK 일 때만 /cmd_vel 을 발행하도록 연동할 것.
"""
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String

try:
    import cv2
    from cv_bridge import CvBridge
    _HAS_CV = True
except Exception:  # 빌드/검출 라이브러리 없을 때도 노드는 떠야 함
    _HAS_CV = False


class ArucoDock(Node):
    def __init__(self):
        super().__init__('hide_aruco_dock')

        self.declare_parameter('target_id', 0)
        self.declare_parameter('stop_distance_m', 0.4)
        self.target_id = self.get_parameter('target_id').value
        self.stop_distance = self.get_parameter('stop_distance_m').value

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.done_pub = self.create_publisher(Bool, '/hide/dock_done', 10)

        self.create_subscription(Image, '/camera/color/image_raw', self._on_image, 1)
        self.create_subscription(CameraInfo, '/camera/color/camera_info', self._on_info, 1)
        self.create_subscription(String, '/hide/state', self._on_state, 10)

        self.state = 'FREEZE'
        self.camera_info = None
        self.docked = False
        self.bridge = CvBridge() if _HAS_CV else None
        if _HAS_CV:
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

        self.get_logger().info(
            f'R2 ArucoDock 시작 (target_id={self.target_id}, '
            f'stop={self.stop_distance}m, cv2={_HAS_CV})')

    def _on_state(self, msg: String):
        new_state = msg.data.strip()
        if new_state != self.state:
            self.docked = False
        self.state = new_state

    def _on_info(self, msg: CameraInfo):
        self.camera_info = msg
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.D = np.array(msg.d, dtype=np.float64)
    def _on_image(self, msg: Image):
        if self.state not in ('RETURN', 'DOCK'):
            return

        if self.docked:
            self.cmd_pub.publish(Twist())
            return

        if not _HAS_CV or self.camera_info is None:
            return

        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        if hasattr(cv2.aruco, 'ArucoDetector'):
            detector = cv2.aruco.ArucoDetector(self.aruco_dict)
            corners, ids, _ = detector.detectMarkers(frame)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(frame, self.aruco_dict)

        if ids is None or self.target_id not in ids.flatten():
            self._tag_visible = False
            self.cmd_pub.publish(Twist())
            return

        if not getattr(self, '_tag_visible', False):
            self.get_logger().info(f'ArUco tag detected: id={self.target_id}')
        self._tag_visible = True

        i = list(ids.flatten()).index(self.target_id)
        marker_len = 0.15

        objp = np.array([
            [-marker_len / 2,  marker_len / 2, 0.0],
            [ marker_len / 2,  marker_len / 2, 0.0],
            [ marker_len / 2, -marker_len / 2, 0.0],
            [-marker_len / 2, -marker_len / 2, 0.0],
        ], dtype=np.float32)

        ok, rvec, tvec = cv2.solvePnP(
            objp, corners[i][0], self.K, self.D,
            flags=cv2.SOLVEPNP_IPPE_SQUARE
        )
        if not ok:
            self.get_logger().warning('ArUco pose estimation failed')
            return

        x = float(tvec[0][0])
        z = float(tvec[2][0])

        rot, _ = cv2.Rodrigues(rvec)
        yaw_deg = float(np.degrees(np.arctan2(rot[1, 0], rot[0, 0])))

        count = getattr(self, '_pose_log_count', 0) + 1
        self._pose_log_count = count
        if count % 30 == 0:
            self.get_logger().info(
                f'ArUco pose: x={x:+.2f} m, z={z:.2f} m, yaw={yaw_deg:+.1f} deg'
            )

        cmd = Twist()

        if z <= self.stop_distance + 0.03 and abs(x) < 0.045:
            self.cmd_pub.publish(cmd)
            self.docked = True
            self.done_pub.publish(Bool(data=True))
            self.get_logger().info(
                f'ArUco 도킹 완료: z={z:.2f}m, x={x:+.3f}m')
            return

        # 태그가 카메라 오른쪽(+x)에 있으면 로봇을 오른쪽으로 회전.
        cmd.angular.z = float(np.clip(-1.8 * x, -0.55, 0.55))

        # 좌우 오차가 큰 경우 회전을 우선하고, 정렬되면 천천히 접근.
        if abs(x) < 0.15 and z > self.stop_distance:
            cmd.linear.x = float(np.clip(
                0.35 * (z - self.stop_distance), 0.03, 0.10))

        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDock()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
