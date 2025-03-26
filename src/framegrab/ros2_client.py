import uuid
from threading import Event

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

bridge = CvBridge()

# start the ROS client if it isn't already started
if not rclpy.ok():
    rclpy.init()


class ROS2Client(Node):
    def __init__(self, topic: str):
        # create a unique node name so that multiple clients can be run simultaneously
        node_name = f"framegrab_node_{uuid.uuid4().hex[:8]}"
        super().__init__(node_name, namespace="groundlight")
        self._msg_event = Event()
        self._latest_msg = None

        self._subscription = self.create_subscription(Image, topic, self._callback, 10)

    def _callback(self, msg: Image) -> None:
        self.get_logger().info("Received a message.")
        self._latest_msg = msg
        self._msg_event.set()

    def grab(self) -> np.ndarray:
        rclpy.spin_once(self, timeout_sec=5.0)
        self.get_logger().info("Waiting for a message...")
        self._msg_event.wait(timeout=5.0)

        if self._latest_msg is None:
            self.get_logger().warn("No message received within timeout.")
            return None

        cv_image = bridge.imgmsg_to_cv2(self._latest_msg, desired_encoding="bgr8")
        return cv_image

    def release(self) -> None:
        self.destroy_node()
