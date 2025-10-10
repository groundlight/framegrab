import argparse
from framegrab import FrameGrabber
from framegrab.config import RTSPFrameGrabberConfig

import cv2
import numpy as np

def resize_frame(frame: np.ndarray, max_width: int = None, max_height: int = None) -> np.ndarray:
    """
    Resizes an image to fit within a given height and/or width, without changing the aspect ratio.
    
    Args:
        frame: Input image as numpy array
        max_width: Maximum width (optional)
        max_height: Maximum height (optional)
    
    Returns:
        Resized image that fits within the specified dimensions
    """
    if max_width is None and max_height is None:
        return frame
    
    height, width = frame.shape[:2]
    
    # Calculate scaling factors
    scale_w = scale_h = 1.0
    
    if max_width is not None and width > max_width:
        scale_w = max_width / width
    
    if max_height is not None and height > max_height:
        scale_h = max_height / height
    
    # Use the smaller scaling factor to ensure image fits within both dimensions
    scale = min(scale_w, scale_h)
    
    # Only resize if scaling is needed
    if scale < 1.0:
        new_width = int(width * scale)
        new_height = int(height * scale)
        return cv2.resize(frame, (new_width, new_height))
    
    return frame

def main():
    parser = argparse.ArgumentParser(description='Stream RTSP video using framegrab')
    parser.add_argument('rtsp_url', help='RTSP URL to stream (e.g., rtsp://localhost:8554/stream_fullsize)')
    args = parser.parse_args()

    config = RTSPFrameGrabberConfig(rtsp_url=args.rtsp_url)
    grabber = FrameGrabber.create_grabber(config)

    print(f"Streaming from: {args.rtsp_url}")
    print("Press 'q' to quit")

    try:
        while True:
            frame = grabber.grab()
            resized_frame = resize_frame(frame, 640, 480) # get a smaller frame so it's easier to view
            cv2.imshow(f'Streaming {args.rtsp_url}', resized_frame)
            key = cv2.waitKey(30)
            if key == ord('q'):
                break
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        grabber.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()