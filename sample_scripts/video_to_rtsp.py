import argparse
from framegrab import FrameGrabber
from framegrab.config import FileStreamFrameGrabberConfig, GenericUSBFrameGrabberConfig

from framegrab.rtsp_server import RTSPServer

import cv2
import numpy as np
import time 

def main():
    parser = argparse.ArgumentParser(description='Stream a video file via RTSP')
    parser.add_argument('video_path', help='Path to the video file to stream')
    parser.add_argument('--port', type=int, default=8554, help='RTSP server port (default: 8554)')
    args = parser.parse_args()

    # Connect to the grabber
    # config = FileStreamFrameGrabberConfig(filename=args.video_path)
    config = GenericUSBFrameGrabberConfig(serial_number='200901010001', resolution_width=1280, resolution_height=720)
    grabber = FrameGrabber.create_grabber(config)

    # Determine the resolution of the video
    test_frame = grabber.grab()
    height, width, _ = test_frame.shape

    # Determine the FPS of the video
    fps = grabber.get_fps()
    
    # Reset to beginning after test frame
    if hasattr(grabber, 'seek_to_beginning'):
        grabber.seek_to_beginning()

    def get_frame_callback() -> np.ndarray:
        try:
            return grabber.grab()
        except RuntimeWarning:
            last_frame_read_number = grabber.get_last_frame_read_number()
            print(f'Got to end of file. Read {last_frame_read_number + 1} frames. Seeking back to the beginning of the video...')
            grabber.seek_to_beginning()
            print(f'Returned to the beginning of the file. Continuing to read the video...')
            return grabber.grab()

    try:
        with RTSPServer(get_frame_callback, width=width, height=height, fps=fps, port=args.port) as server:
            print(server)
            print("Press Ctrl+C to stop...")
            
            while True:
                time.sleep(1)  # Keep alive, wake up periodically to check for KeyboardInterrupt
                
    except KeyboardInterrupt:
        print("\nShutting down gracefully...")
    finally:
        grabber.release()

if __name__ == "__main__":
    main()