#!/usr/bin/env python3
"""Reactive obstacle-avoidance wandering node for Storagy (lightweight test).

Subscribes to /scan (LaserScan), publishes /cmd_vel (Twist).
Drives forward while the front is clear; when blocked, stops and turns
toward whichever side (left/right) has more open space.
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class WanderNode(Node):
    def __init__(self):
        super().__init__('wander_node')

        # Parameters (override via launch or `ros2 param`).
        self.declare_parameter('safe_distance', 0.6)        # [m] obstacle threshold ahead
        self.declare_parameter('linear_speed', 0.15)        # [m/s] forward speed
        self.declare_parameter('angular_speed', 0.5)        # [rad/s] turn speed
        self.declare_parameter('front_angle_deg', 30.0)     # [deg] half-width of front sector
        self.declare_parameter('scan_timeout', 0.5)         # [s] stop if no scan within this
        self.declare_parameter('control_period', 0.1)       # [s] control loop period

        self.safe_distance = self.get_parameter('safe_distance').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.front_angle = math.radians(self.get_parameter('front_angle_deg').value)
        self.scan_timeout = self.get_parameter('scan_timeout').value
        control_period = self.get_parameter('control_period').value

        self.last_scan = None
        self.last_scan_time = None
        # Latch turn direction during a turn so the robot doesn't dither: +1 left, -1 right.
        self.turn_dir = 0

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_cb, 10)
        self.timer = self.create_timer(control_period, self.control_loop)

        self.get_logger().info(
            f'wander_node started: safe_distance={self.safe_distance} m, '
            f'linear={self.linear_speed} m/s, angular={self.angular_speed} rad/s')

    def scan_cb(self, msg: LaserScan):
        self.last_scan = msg
        self.last_scan_time = self.get_clock().now()

    def sector_min(self, scan: LaserScan, center: float, half_width: float):
        """Minimum valid range within [center-half_width, center+half_width] radians.

        Angles are relative to the scanner's forward axis (0 rad). Returns inf
        if no valid (finite, in-range) measurement falls in the sector.
        """
        best = math.inf
        angle = scan.angle_min
        for r in scan.ranges:
            # Normalize angle difference to [-pi, pi].
            d = math.atan2(math.sin(angle - center), math.cos(angle - center))
            if abs(d) <= half_width:
                if (math.isfinite(r)
                        and scan.range_min <= r <= scan.range_max):
                    if r < best:
                        best = r
            angle += scan.angle_increment
        return best

    def control_loop(self):
        twist = Twist()

        # Safety: no recent scan -> stop.
        if self.last_scan is None or self.last_scan_time is None:
            self.cmd_pub.publish(twist)
            return
        age = (self.get_clock().now() - self.last_scan_time).nanoseconds * 1e-9
        if age > self.scan_timeout:
            self.get_logger().warn('scan timeout, stopping', throttle_duration_sec=2.0)
            self.cmd_pub.publish(twist)
            return

        scan = self.last_scan
        front = self.sector_min(scan, 0.0, self.front_angle)

        if front > self.safe_distance:
            # Path ahead clear: go straight, reset latched turn.
            self.turn_dir = 0
            twist.linear.x = self.linear_speed
        else:
            # Blocked: pick a turn direction once, then keep turning until clear.
            if self.turn_dir == 0:
                left = self.sector_min(scan, math.radians(90.0), self.front_angle)
                right = self.sector_min(scan, math.radians(-90.0), self.front_angle)
                self.turn_dir = 1 if left >= right else -1
            twist.angular.z = self.angular_speed * self.turn_dir

        self.cmd_pub.publish(twist)

    def stop(self):
        try:
            self.cmd_pub.publish(Twist())
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = WanderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
