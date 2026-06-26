#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from enum import Enum
import math
import random

class RobotState(Enum):
    WANDERING = 1
    NAVIGATING = 2
    WAITING = 3

class WanderNavNode(Node):
    def __init__(self):
        super().__init__('wander_nav_node')
        
        self.state = RobotState.WANDERING
        self.wait_start_time = None
        self.obstacle_detected = False
        self.obstacle_distance = float('inf')
        self.turn_direction = 1.0  # 1.0 for left, -1.0 for right
        self.linear_speed = 0.2
        self.angular_speed = 0.5
        
        # Subscriptions
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )
        
        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Action Client for Nav2
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Control Loop Timer (10Hz)
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info("Wander & Navigation Node initialized in WANDERING mode.")

    def scan_callback(self, msg: LaserScan):
        # We check the front cone: -45 degrees to +45 degrees (approx -0.78 to +0.78 rad)
        # Find min range in this front cone
        front_angles_min = -0.78
        front_angles_max = 0.78
        
        min_distance = float('inf')
        left_min_distance = float('inf')
        right_min_distance = float('inf')
        
        for i, dist in enumerate(msg.ranges):
            # Skip invalid values
            if math.isnan(dist) or math.isinf(dist) or dist < msg.range_min or dist > msg.range_max:
                continue
                
            angle = msg.angle_min + i * msg.angle_increment
            
            # Normalize angle to [-pi, pi]
            angle = math.atan2(math.sin(angle), math.cos(angle))
            
            if front_angles_min <= angle <= front_angles_max:
                if dist < min_distance:
                    min_distance = dist
                
                # Split front cone to left and right for directional turning decision
                if angle >= 0:
                    if dist < left_min_distance:
                        left_min_distance = dist
                else:
                    if dist < right_min_distance:
                        right_min_distance = dist
        
        # If any object is closer than 0.8m, flag obstacle
        was_obstacle = self.obstacle_detected
        if min_distance < 0.8:
            self.obstacle_detected = True
            self.obstacle_distance = min_distance
            # Only choose/lock turn direction if it's a new obstacle detection to prevent oscillation
            if not was_obstacle:
                if left_min_distance > right_min_distance:
                    self.turn_direction = 1.0  # Turn left
                else:
                    self.turn_direction = -1.0  # Turn right
        else:
            self.obstacle_detected = False
            self.obstacle_distance = float('inf')

    def goal_callback(self, msg: PoseStamped):
        # Intercept goal pose and switch state
        if self.state in [RobotState.WANDERING, RobotState.WAITING]:
            self.get_logger().info(f"Received goal pose: {msg.pose.position.x}, {msg.pose.position.y}. Switching to NAVIGATING state.")
            self.state = RobotState.NAVIGATING
            self.send_nav_goal(msg)

    def send_nav_goal(self, pose: PoseStamped):
        self.get_logger().info("Waiting for NavigateToPose action server...")
        self.nav_client.wait_for_server()
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        
        self.get_logger().info("Sending goal to Nav2...")
        self.send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.nav_feedback_callback
        )
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def nav_feedback_callback(self, feedback_msg):
        pass

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected by Nav2. Returning to WAITING state.")
            self.start_waiting_state()
            return
            
        self.get_logger().info("Goal accepted by Nav2, navigating...")
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result()
        status = result.status
        self.get_logger().info(f"Navigation finished with status code: {status}. Entering 5 seconds WAITING state.")
        self.start_waiting_state()

    def start_waiting_state(self):
        self.state = RobotState.WAITING
        self.wait_start_time = self.get_clock().now()
        
        # Stop the robot immediately
        stop_twist = Twist()
        self.cmd_vel_pub.publish(stop_twist)

    def control_loop(self):
        if self.state == RobotState.WANDERING:
            twist = Twist()
            if self.obstacle_detected:
                if self.obstacle_distance < 0.45:
                    # Too close to obstacle/corner! Back up and rotate opposite to get out
                    twist.linear.x = -0.12
                    twist.angular.z = -0.3 * self.turn_direction
                else:
                    # Rotate in place to avoid obstacle
                    twist.linear.x = 0.0
                    twist.angular.z = self.angular_speed * self.turn_direction
            else:
                # Move forward with some random gentle steering
                twist.linear.x = self.linear_speed
                twist.angular.z = random.uniform(-0.1, 0.1)
                
            self.cmd_vel_pub.publish(twist)
            
        elif self.state == RobotState.WAITING:
            stop_twist = Twist()
            self.cmd_vel_pub.publish(stop_twist)
            
            if self.wait_start_time is not None:
                elapsed = (self.get_clock().now() - self.wait_start_time).nanoseconds / 1e9
                if elapsed >= 5.0:
                    self.get_logger().info("5 seconds wait completed. Switching back to WANDERING state.")
                    self.state = RobotState.WANDERING

        elif self.state == RobotState.NAVIGATING:
            # In NAVIGATING state, we let Nav2 publish velocity commands
            pass

def main(args=None):
    rclpy.init(args=args)
    node = WanderNavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received, shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
