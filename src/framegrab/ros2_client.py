import uuid
from threading import Event

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image

SUPPORTED_IMAGE_TYPES = (
    "sensor_msgs/msg/CompressedImage",
    "sensor_msgs/msg/Image",
)

# start the ROS client if it isn't already started
if not rclpy.ok():
    rclpy.init()


class ROS2Client(Node):
    def __init__(self, topic: str):
        # create a unique node name so that multiple clients can be run simultaneously
        # node_name = f"framegrab_node_{uuid.uuid4().hex[:8]}"
        node_name = topic.replace("/", "_").lstrip("_") + f"_{uuid.uuid4().hex[:8]}"
        # node_name = topic.replace('/', '')
        super().__init__(node_name, namespace="groundlight")
        self._msg_event = Event()
        self._latest_msg = None

        # Validate the topic type and create the subscription
        topic_list = self.get_topic_names_and_types()
        available_topics = []
        for name, types in topic_list:
            type_ = types[0]

            if type_ in SUPPORTED_IMAGE_TYPES:
                available_topics.append(name)

            if name != topic:
                continue

            if type_ == "sensor_msgs/msg/CompressedImage":
                print("sensor_msgs/msg/CompressedImage")
                self._subscription = self.create_subscription(CompressedImage, topic, self._image_callback, 10)
                break
            elif type_ == "sensor_msgs/msg/Image":
                print("sensor_msgs/msg/Image")
                self._subscription = self.create_subscription(Image, topic, self._image_callback, 10)
                break
            else:
                raise ValueError(
                    f"Requested topic {topic} is of type {type_}, which is not a supported type. "
                    f"Supported types are {SUPPORTED_IMAGE_TYPES}."
                )
        else:
            raise ValueError(f"Requested topic {topic} was not found. Available topics are {available_topics}.")

    def _image_callback(self, msg: Image | CompressedImage) -> None:
        self._latest_msg = msg
        self._msg_event.set()

    def grab(self) -> np.ndarray:
        rclpy.spin_once(self, timeout_sec=1.0)
        self.get_logger().debug("Waiting for a message...")
        self._msg_event.wait(timeout=1.0)

        if self._latest_msg is None:
            self.get_logger().warn("No message received within timeout.")
            return None

        if isinstance(self._latest_msg, CompressedImage):
            np_arr = np.frombuffer(self._latest_msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if cv_image is None:
                self.get_logger().error("Failed to decode compressed image.")
            return cv_image

        elif isinstance(self._latest_msg, Image):
            img = np.frombuffer(self._latest_msg.data, dtype=np.uint8)
            try:
                cv_image = img.reshape((self._latest_msg.height, self._latest_msg.width, 3))
            except ValueError as e:
                self.get_logger().error(f"Image reshape failed: {e}")
                return None
            return cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)

        else:
            self.get_logger().error(f"Unsupported message type: {type(self._latest_msg)}")
            return None

    def release(self) -> None:
        self.destroy_node()
