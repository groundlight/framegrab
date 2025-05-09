import time
import uuid
from threading import Event
from typing import Callable, Dict, List, Tuple

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image

ROS_NODE_NAMESPACE = "groundlight"
SUPPORTED_IMAGE_TYPES = (
    "sensor_msgs/msg/CompressedImage",
    "sensor_msgs/msg/Image",
)


def _topic_discovery_retry(
    func: Callable[[], List[Tuple[str, List[str]]]], attempts: int = 10, delay: float = 0.01
) -> List[Tuple[str, List[str]]]:
    """
    Topic discovery can be flakey in ROS.

    Wrap topic discovery (Node.get_topic_names_and_types) in this function to get more robust results.
    """

    topic_dict: Dict[str, set] = {}

    for _ in range(attempts):
        topics = func()
        for name, types in topics:
            if name not in topic_dict:
                topic_dict[name] = set()
            topic_dict[name].update(types)
        time.sleep(delay)

    # Convert the merged dictionary back to a list of tuples
    merged_topics = [(name, list(types)) for name, types in topic_dict.items()]
    return merged_topics


# start the ROS client if it isn't already started
if not rclpy.ok():
    rclpy.init()


class ROS2Client(Node):
    @staticmethod
    def discover_topics() -> list[str]:
        """
        Returns a list of available ROS 2 image topics that are of a supported type
        """
        node = Node("topic_discovery", namespace=ROS_NODE_NAMESPACE)
        topic_list = _topic_discovery_retry(node.get_topic_names_and_types)
        node.destroy_node()

        available_topics = []
        for name, types in topic_list:
            type_ = types[0]  # assuming that each topic only has one type, which should be okay for image topics
            if type_ in SUPPORTED_IMAGE_TYPES:
                available_topics.append(name)

        return available_topics

    def __init__(self, topic: str):
        # create a unique node name so that multiple clients can be run simultaneously
        node_name = topic.replace("/", "_").lstrip("_") + f"_{uuid.uuid4().hex[:8]}"
        super().__init__(node_name, namespace=ROS_NODE_NAMESPACE)

        # Track the lastest message received by the subscriber, and use an Event to signal a new frame is requested.
        # This ensures that when `grab` is called, the next available frame will be returned, instead
        # of a previous frame, which could potentially be stale.
        self._msg_event = Event()
        self._latest_msg = None

        # Validate the topic type and create the subscription
        full_topic_list = _topic_discovery_retry(self.get_topic_names_and_types)
        for name, types in full_topic_list:
            if name != topic:
                continue

            type_ = types[0]
            if type_ == "sensor_msgs/msg/CompressedImage":
                image_type = CompressedImage
                break
            elif type_ == "sensor_msgs/msg/Image":
                image_type = Image
                break
            else:
                raise ValueError(
                    f"Requested topic {topic} is of type {type_}, which is not a supported type. "
                    f"Supported types are {SUPPORTED_IMAGE_TYPES}."
                )
        else:
            raise ValueError(f"Requested topic {topic} was not found.")

        self._subscription = self.create_subscription(image_type, topic, self._image_callback, 1)

    def _image_callback(self, msg: Image | CompressedImage) -> None:
        self._latest_msg = msg
        self._msg_event.set()

    def grab(self) -> np.ndarray | None:
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
