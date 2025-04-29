import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
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
        self.publisher = self.create_publisher(CompressedImage, 'sample_image/compressed', 10)
        timer_period = 1 / 15
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        resolved_topic = self.resolve_topic_name(self.publisher.topic)
        self.get_logger().info(f"Publishing images to {resolved_topic}...")

    def timer_callback(self):
        image = generate_image()

        # Encode the image as JPEG
        success, encoded_image = cv2.imencode('.jpg', image)
        if not success:
            self.get_logger().error("Failed to encode image")
            return

        # Create a CompressedImage message
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = 'jpeg'
        msg.data = encoded_image.tobytes()

        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SampleCameraPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
