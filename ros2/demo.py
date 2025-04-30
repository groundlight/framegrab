import argparse
from framegrab import FrameGrabber
from framegrab.ros2_client import discover_topics
import cv2

def main():
    parser = argparse.ArgumentParser(description="FrameGrab demo for ROS2 cameras.")
    parser.add_argument(
        "--topic",
        type=str,
        default="",
    )
    args = parser.parse_args()
    
    if not args.topic:
        available_topics = discover_topics()
        raise ValueError(
            f'Please provide a --topic argument. Available topics are: {available_topics}'
        )

    config = {
        'input_type': 'ros2',
        'name': 'My ROS2 Camera',
        'id': {
            'topic': args.topic
        },
    }

    grabber = FrameGrabber.create_grabber(config)
    frame = grabber.grab()
    print(
        'Demo started. Press "q" to quit. '
        'Press "r" to rotate the image. '
        'Press any other key to grab the next frame. '
        'Hold any other key to stream video.'
    )

    num_90_deg_rotations = 0

    while True:
        if frame is None:
            print('No frame received.')
            break

        cv2.imshow('FrameGrab Image', frame)
        key = cv2.waitKey(0)
        if key == ord('q'):
            break
        elif key == ord('r'):
            num_90_deg_rotations = (num_90_deg_rotations + 1) % 4

        options = {
            'num_90_deg_rotations': num_90_deg_rotations
        }
        grabber.apply_options(options)

        frame = grabber.grab()

    grabber.release()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
