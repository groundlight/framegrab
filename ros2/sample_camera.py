import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
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
        self.pub_compressed = self.create_publisher(CompressedImage, 'sample_image/compressed', 10)
        self.pub_uncompressed = self.create_publisher(Image, 'sample_image/raw', 10)

        self.timer = self.create_timer(1 / 15, self.timer_callback)

        self.get_logger().info(f"Publishing to topics:\n"
                               f" - {self.resolve_topic_name(self.pub_compressed.topic)} (compressed)\n"
                               f" - {self.resolve_topic_name(self.pub_uncompressed.topic)} (uncompressed)")

    def timer_callback(self):
        image = generate_image()
        now = self.get_clock().now().to_msg()

        # Compressed
        success, encoded_image = cv2.imencode('.jpg', image)
        if not success:
            self.get_logger().error("Failed to encode image")
            return

        msg_compressed = CompressedImage()
        msg_compressed.header.stamp = now
        msg_compressed.format = 'jpeg'
        msg_compressed.data = encoded_image.tobytes()
        self.pub_compressed.publish(msg_compressed)

        # Uncompressed
        msg_raw = Image()
        msg_raw.header.stamp = now
        msg_raw.height = image.shape[0]
        msg_raw.width = image.shape[1]
        msg_raw.encoding = 'rgb8'
        msg_raw.is_bigendian = False
        msg_raw.step = msg_raw.width * 3
        # Convert BGR (OpenCV default) to RGB for ROS
        rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        msg_raw.data = rgb_image.tobytes()
        self.pub_uncompressed.publish(msg_raw)

def main(args=None):
    rclpy.init(args=args)
    node = SampleCameraPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()