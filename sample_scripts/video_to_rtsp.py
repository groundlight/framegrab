import argparse
import logging
import time
from framegrab import FrameGrabber
from framegrab.config import FileStreamFrameGrabberConfig

from framegrab.rtsp_server import RTSPServer

import cv2
import numpy as np

# Configure logging to show INFO level messages
logging.basicConfig(level=logging.INFO, format='%(levelname)s - %(name)s - %(message)s')

logger = logging.getLogger(__name__)

SMALL_FRAME_WIDTH_MAX = 300
SMALL_FRAME_HEIGHT_MAX = 200

class VideoToRTSPSampleApp:
    def __init__(self, video_paths: list[str], port: int):

        self.video_paths = video_paths
        self.port = port

        self.server = RTSPServer(port=port)
        
        self.grabbers = []
        for n, video_path in enumerate(video_paths):
            # Connect to the grabber
            config = FileStreamFrameGrabberConfig(filename=video_path)
            grabber = FrameGrabber.create_grabber(config)
            self.grabbers.append(grabber)

            # Determine the resolution of the video
            test_frame = grabber.grab()
            height, width, _ = test_frame.shape

            # Determine the FPS of the video
            fps = grabber.get_fps()
            
            # Reset to beginning after test frame
            grabber.seek_to_beginning()

            def get_frame_callback(grabber: FrameGrabber = grabber, video_path: str = video_path) -> np.ndarray:
                try:
                    return grabber.grab()
                except RuntimeWarning:
                    last_frame_read_number = grabber.get_last_frame_read_number()
                    logger.info(f'Got to end of {video_path}. Read {last_frame_read_number + 1} frames. Restarting from the beginning of the video...')
                    grabber.seek_to_beginning()
                    return grabber.grab()

            self.server.create_stream(get_frame_callback, width=width, height=height, fps=fps, mount_point=f'/stream{n}')

    def list_rtsp_urls(self) -> list[str]:
        return self.server.list_rtsp_urls()

    def run(self) -> None:
        self.server.start()

    def stop(self) -> None:
        self.server.stop()

        for g in self.grabbers:
            g.release()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Stream multiple video files via RTSP')
    parser.add_argument('video_paths', nargs='+', help='Paths to video files to stream (one or more)')
    parser.add_argument('--port', type=int, default=8554, help='RTSP server port (default: 8554)')
    args = parser.parse_args()

    app = VideoToRTSPSampleApp(args.video_paths, args.port)

    try:
        app.run()
        logger.info(f'RTSP Server started on port {app.port}')

        rtsp_urls = app.list_rtsp_urls()
        for url, path in zip(rtsp_urls, app.video_paths):
            logger.info(f'{path} available at {url}')
        logger.info("Press Ctrl+C to stop...")
        
        # Keep the program running
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        logger.info("Shutting down gracefully...")
    finally:
        app.stop()