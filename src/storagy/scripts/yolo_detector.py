#!/usr/bin/env python3
"""
YOLO Person Detection Node for Storagy Robot.

Subscribes to the robot's camera image, runs YOLOv8 inference to detect people,
and publishes the annotated image for viewing in RViz.

Usage:
  1. Start simulation:  ros2 launch storagy simulation_bringup.launch.py ...
  2. Run this node:     python3 src/storagy/scripts/yolo_detector.py
  3. In RViz, add Image display and set topic to /yolo/detected_image
"""

import sys
import os

# ultralytics & torch are installed into the system Python environment
# (see requirements.txt / Dockerfile). For local development outside Docker
# you can still point this at a virtualenv's site-packages via the optional
# YOLO_ENV_SITE environment variable.
_yolo_env_site = os.environ.get("YOLO_ENV_SITE")
if _yolo_env_site and _yolo_env_site not in sys.path:
    sys.path.insert(0, _yolo_env_site)

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from cv_bridge import CvBridge

from ultralytics import YOLO


class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__("yolo_detector")

        # Parameters
        self.declare_parameter("model", "yolov8n.pt")
        self.declare_parameter("confidence", 0.7)
        self.declare_parameter("input_topic", "/camera/color/image_raw")
        self.declare_parameter("output_topic", "/yolo/detected_image")

        model_name = self.get_parameter("model").get_parameter_value().string_value
        self.conf_threshold = self.get_parameter("confidence").get_parameter_value().double_value
        input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value

        # Load YOLO model
        self.get_logger().info(f"Loading YOLO model: {model_name}")
        self.model = YOLO(model_name)
        self.get_logger().info("YOLO model loaded successfully!")

        self.bridge = CvBridge()

        # Match ros_gz_bridge camera (RELIABLE) and web_dashboard subscriber.
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Subscriber
        self.sub = self.create_subscription(
            Image, input_topic, self.image_callback, image_qos
        )

        # Publisher
        self.pub = self.create_publisher(Image, output_topic, image_qos)
        self.count_pub = self.create_publisher(Int32, "/yolo/person_count", 10)

        self.get_logger().info(
            f"YOLO Detector ready! Subscribing: {input_topic} -> Publishing: {output_topic}"
        )
        self.get_logger().info(
            f"Confidence threshold: {self.conf_threshold}"
        )

    def image_callback(self, msg: Image):
        try:
            # Convert ROS Image -> OpenCV BGR
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return

        # Run YOLO inference (only detect 'person' class = 0)
        results = self.model(
            cv_image,
            conf=self.conf_threshold,
            classes=[0],  # 0 = person in COCO
            verbose=False,
        )

        # Draw results on image
        annotated = results[0].plot()

        # Count detections
        num_people = len(results[0].boxes)
        self.count_pub.publish(Int32(data=num_people))
        if num_people > 0:
            self.get_logger().info(
                f"Detected {num_people} person(s)", throttle_duration_sec=2.0
            )

        # Convert back to ROS Image and publish
        try:
            out_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            out_msg.header = msg.header
            self.pub.publish(out_msg)
        except Exception as e:
            self.get_logger().error(f"Publish error: {e}")


def main():
    rclpy.init()
    node = YoloDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down YOLO detector...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
