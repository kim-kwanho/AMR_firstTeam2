#!/usr/bin/env python3
"""P3(파트 B) — 동적 코스트맵 주입 (인간 시야 부채꼴 → 진입금지 마스크).

토이 가이드(컨셉 3): 손님 전방 120도 부채꼴(/hide/keepout_zones)을 OccupancyGrid 마스크로
래스터화해 발행한다. Nav2 의 KeepoutFilter 가 이 토픽을 mask 로 구독하면 로봇이
"인간 시야"를 가상의 벽으로 인식해 구석 사각지대로 회피한다.

토픽 계약:
  - 구독: /hide/keepout_zones(PolygonStamped)
  - 발행: /hide/dynamic_keepout_mask(OccupancyGrid)
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PolygonStamped
from nav_msgs.msg import OccupancyGrid

try:
    import cv2
    _HAS_CV = True
except Exception:
    _HAS_CV = False


class DynamicCostmap(Node):
    def __init__(self):
        super().__init__('hide_dynamic_costmap')

        self.declare_parameter('resolution_m', 0.05)
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('padding_m', 0.1)
        self.declare_parameter('occupied_value', 100)
        self.resolution = self.get_parameter('resolution_m').value
        self.frame_id = self.get_parameter('frame_id').value
        self.padding = self.get_parameter('padding_m').value
        self.occupied = int(self.get_parameter('occupied_value').value)

        self.mask_pub = self.create_publisher(
            OccupancyGrid, '/hide/dynamic_keepout_mask', 1)
        self.create_subscription(
            PolygonStamped, '/hide/keepout_zones', self._on_zones, 10)

        if not _HAS_CV:
            self.get_logger().warn('cv2 미설치 → 폴리곤 바운딩박스 전체를 채우는 폴백 사용')
        self.get_logger().info('P3 DynamicCostmap 시작')

    def _on_zones(self, msg: PolygonStamped):
        pts = [(p.x, p.y) for p in msg.polygon.points]
        if len(pts) < 3:
            return

        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        res = self.resolution
        min_x, max_x = min(xs) - self.padding, max(xs) + self.padding
        min_y, max_y = min(ys) - self.padding, max(ys) + self.padding

        width = max(1, int(math.ceil((max_x - min_x) / res)))
        height = max(1, int(math.ceil((max_y - min_y) / res)))

        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = msg.header.frame_id or self.frame_id
        grid.info.resolution = res
        grid.info.width = width
        grid.info.height = height
        grid.info.origin.position.x = float(min_x)
        grid.info.origin.position.y = float(min_y)
        grid.info.origin.orientation.w = 1.0

        data = np.zeros((height, width), dtype=np.int8)
        poly_px = np.array(
            [[int((x - min_x) / res), int((y - min_y) / res)] for x, y in pts],
            dtype=np.int32)

        if _HAS_CV:
            filled = np.zeros((height, width), dtype=np.uint8)
            cv2.fillPoly(filled, [poly_px], 1)
            data[filled > 0] = self.occupied
        else:
            data[:, :] = self.occupied

        grid.data = data.flatten().tolist()
        self.mask_pub.publish(grid)


def main(args=None):
    rclpy.init(args=args)
    node = DynamicCostmap()
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
