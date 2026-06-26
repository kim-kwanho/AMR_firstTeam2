#!/usr/bin/env python3

import math
from enum import Enum

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose2D, PoseStamped, Quaternion, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool, String


class GuideState(str, Enum):
    IDLE = 'IDLE'
    GUIDE_READY = 'GUIDE_READY'
    GUIDING = 'GUIDING'
    PAUSED = 'PAUSED'
    ARRIVED = 'ARRIVED'
    FAILED = 'FAILED'


def quaternion_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ('1', 'true', 'yes', 'on')
    return bool(value)


class GuideNavNode(Node):
    """MVP guide-side Nav2 client.

    A simple "person arrived" signal starts navigation to a fixed map pose.
    Freeze cancels the active goal and repeatedly publishes zero cmd_vel.
    """

    def __init__(self):
        super().__init__('guide_nav_node')

        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('target_x', 1.0)
        self.declare_parameter('target_y', 0.0)
        self.declare_parameter('target_yaw', 0.0)
        self.declare_parameter('demo_person_arrived', False)
        self.declare_parameter('resend_goal_after_resume', True)
        self.declare_parameter('stop_publish_hz', 10.0)

        self.state = GuideState.IDLE
        self.freeze = False
        self.person_arrived = False
        self.goal_active = False
        self.goal_done = False
        self.goal_handle = None
        self.latest_line_error = Pose2D()

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.state_pub = self.create_publisher(String, '/guide/state', 10)
        self.mission_done_pub = self.create_publisher(Bool, '/guide/mission_done', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_subscription(Bool, '/guide/person_arrived',
                                 self.person_arrived_callback, 10)
        self.create_subscription(String, '/guide/command',
                                 self.command_callback, 10)
        self.create_subscription(Bool, '/hide/freeze',
                                 self.freeze_callback, 10)
        self.create_subscription(Pose2D, '/guide/yellow_line_error',
                                 self.line_error_callback, 10)

        stop_hz = max(1.0, float(self.get_parameter('stop_publish_hz').value))
        self.stop_timer = self.create_timer(1.0 / stop_hz, self.stop_timer_callback)
        self.demo_timer = self.create_timer(0.5, self.demo_timer_callback)

        self.set_state(GuideState.IDLE)
        self.get_logger().info(
            'GuideNavNode ready. Trigger with /guide/person_arrived=true, '
            '/guide/command=start_guide, or demo_person_arrived:=true.'
        )

    def set_state(self, state: GuideState):
        if self.state == state:
            return
        self.state = state
        msg = String()
        msg.data = state.value
        self.state_pub.publish(msg)
        self.get_logger().info(f'/guide/state -> {msg.data}')

    def demo_timer_callback(self):
        if as_bool(self.get_parameter('demo_person_arrived').value):
            self.person_arrived = True
            if not self.goal_active and not self.goal_done and not self.freeze:
                self.get_logger().info('demo_person_arrived is true; starting guide goal.')
                self.start_fixed_goal()

    def person_arrived_callback(self, msg: Bool):
        self.person_arrived = msg.data
        if msg.data:
            self.get_logger().info('Person-arrived signal received.')
            if not self.goal_active and not self.goal_done and not self.freeze:
                self.start_fixed_goal()

    def command_callback(self, msg: String):
        command = msg.data.strip().lower()
        if command in ('start', 'start_guide', 'guide'):
            self.person_arrived = True
            if not self.freeze:
                self.start_fixed_goal()
        elif command == 'resume':
            self.freeze_callback(Bool(data=False))
        elif command in ('stop', 'pause', 'freeze'):
            self.freeze_callback(Bool(data=True))

    def freeze_callback(self, msg: Bool):
        if self.freeze == msg.data:
            return
        self.freeze = msg.data
        if self.freeze:
            self.get_logger().warn('/hide/freeze true: canceling guide goal and stopping.')
            self.cancel_active_goal()
            self.publish_stop()
            self.set_state(GuideState.PAUSED)
            return

        self.get_logger().info('/hide/freeze false: guide can resume.')
        if (self.person_arrived and not self.goal_done and
                as_bool(self.get_parameter('resend_goal_after_resume').value)):
            self.start_fixed_goal()
        else:
            self.set_state(GuideState.GUIDE_READY)

    def line_error_callback(self, msg: Pose2D):
        self.latest_line_error = msg
        self.get_logger().debug(
            f'/guide/yellow_line_error y={msg.y:.3f}, theta={msg.theta:.3f}'
        )

    def stop_timer_callback(self):
        if self.freeze:
            self.publish_stop()

    def publish_stop(self):
        self.cmd_vel_pub.publish(Twist())

    def fixed_target_pose(self) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = str(self.get_parameter('target_frame').value)
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(self.get_parameter('target_x').value)
        pose.pose.position.y = float(self.get_parameter('target_y').value)
        pose.pose.orientation = quaternion_from_yaw(
            float(self.get_parameter('target_yaw').value)
        )
        return pose

    def start_fixed_goal(self):
        if self.freeze:
            self.get_logger().warn('Guide goal blocked because /hide/freeze is true.')
            self.set_state(GuideState.PAUSED)
            return
        if self.goal_active:
            self.get_logger().info('Guide goal already active; ignoring duplicate start.')
            return

        self.set_state(GuideState.GUIDE_READY)
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Nav2 navigate_to_pose action server is not ready.')
            self.set_state(GuideState.FAILED)
            return

        goal = NavigateToPose.Goal()
        goal.pose = self.fixed_target_pose()
        self.get_logger().info(
            'Sending guide goal: '
            f'x={goal.pose.pose.position.x:.2f}, '
            f'y={goal.pose.pose.position.y:.2f}'
        )
        self.goal_active = True
        self.goal_done = False
        send_future = self.nav_client.send_goal_async(
            goal, feedback_callback=self.feedback_callback)
        send_future.add_done_callback(self.goal_response_callback)
        self.set_state(GuideState.GUIDING)

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().error('Guide goal rejected by Nav2.')
            self.goal_active = False
            self.set_state(GuideState.FAILED)
            return
        self.get_logger().info('Guide goal accepted by Nav2.')
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def feedback_callback(self, feedback_msg):
        distance = feedback_msg.feedback.distance_remaining
        self.get_logger().debug(f'Guide distance remaining: {distance:.2f} m')

    def goal_result_callback(self, future):
        status = future.result().status
        self.goal_active = False
        self.goal_handle = None

        if self.freeze:
            self.get_logger().info('Guide goal result ignored while paused.')
            return
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.goal_done = True
            self.set_state(GuideState.ARRIVED)
            done = Bool()
            done.data = True
            self.mission_done_pub.publish(done)
            self.get_logger().info('/guide/mission_done -> true')
            return

        self.get_logger().warn(f'Guide goal finished with status={status}.')
        self.set_state(GuideState.FAILED)

    def cancel_active_goal(self):
        if self.goal_handle is None:
            self.goal_active = False
            return
        cancel_future = self.goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(
            lambda _future: self.get_logger().info('Cancel request sent to Nav2.'))
        self.goal_active = False
        self.goal_handle = None


def main(args=None):
    rclpy.init(args=args)
    node = GuideNavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
