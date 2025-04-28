import uuid
from threading import Event

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import cv2

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

        self._subscription = self.create_subscription(CompressedImage, topic, self._callback, 10)

    def _callback(self, msg: CompressedImage) -> None:
        self.get_logger().debug("Received a message.")
        self._latest_msg = msg
        self._msg_event.set()

    def grab(self) -> np.ndarray:
        rclpy.spin_once(self, timeout_sec=5.0)
        self.get_logger().debug("Waiting for a message...")
        self._msg_event.wait(timeout=5.0)

        if self._latest_msg is None:
            self.get_logger().warn("No message received within timeout.")
            return None

        # Decode the compressed image data to a cv2 image
        np_arr = np.frombuffer(self._latest_msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if cv_image is None:
            self.get_logger().error("Failed to decode compressed image.")
        return cv_image

    def release(self) -> None:
        self.destroy_node()
