import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
from datetime import datetime
import cv2

def generate_image(width: int = 640, height: int = 480) -> np.ndarray:
    # Create static/noise image
    image = np.random.randint(0, 256, (height, width, 3), dtype=np.uint8)

    # Get current timestamp
    timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    # Choose font and scale
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 1
    font_thickness = 2

    # Get text size
    (text_width, text_height), baseline = cv2.getTextSize(timestamp, font, font_scale, font_thickness)

    # Calculate center coordinates
    x = (width - text_width) // 2
    y = (height + text_height) // 2

    # Draw black rectangle behind text
    cv2.rectangle(image, (x - 10, y - text_height - 10), (x + text_width + 10, y + baseline + 10), (0, 0, 0), thickness=-1)

    # Put white text over the rectangle
    cv2.putText(image, timestamp, (x, y), font, font_scale, (255, 255, 255), font_thickness, cv2.LINE_AA)

    return image

class SampleCameraPublisher(Node):
    def __init__(self):
        super().__init__('sample_camera_publisher', namespace='groundlight')
        self.publisher_ = self.create_publisher(Image, 'sample_image', 10)
        self.bridge = CvBridge()
        timer_period = 1 / 15
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info("Static image publisher started")

    def timer_callback(self):
        image = generate_image()
        msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
        self.publisher_.publish(msg)
        self.get_logger().info("Published an image")

def main(args=None):
    rclpy.init(args=args)
    node = SampleCameraPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
