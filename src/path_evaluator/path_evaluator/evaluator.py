import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
import math


class PathEvaluator(Node):

    def __init__(self):
        super().__init__('path_evaluator')

        self.subscription = self.create_subscription(
            Path,
            '/plan',
            self.path_callback,
            10
        )

    def path_callback(self, msg):

        length = 0.0
        turn_cost = 0.0
        headings = []

        for i in range(1, len(msg.poses)):
    
            p1 = msg.poses[i-1].pose.position
            p2 = msg.poses[i].pose.position
    
            dx = p2.x - p1.x
            dy = p2.y - p1.y

            # 거리 계산
            length += math.sqrt(dx**2 + dy**2)
  
            # 방향 계산
            heading = math.atan2(dy, dx)
            headings.append(heading)

        # 회전량 계산
        for i in range(1, len(headings)):

            diff = abs(headings[i] - headings[i-1])

            # 각도 정규화
            if diff > math.pi:
                diff = 2 * math.pi - diff

            turn_cost += diff

        self.get_logger().info(
            f'Path Length : {length:.2f} m | Turn Cost : {turn_cost:.2f} rad'
        )


def main():

    rclpy.init()

    node = PathEvaluator()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()