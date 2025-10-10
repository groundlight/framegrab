import unittest
import numpy as np
import time

from framegrab.rtsp_server import RTSPServer
from framegrab.grabber import RTSPFrameGrabber
from framegrab.config import RTSPFrameGrabberConfig

import time

def generate_noise_frame(width: int, height: int) -> np.ndarray:
    return np.random.randint(0, 255, (height, width, 3), dtype=np.uint8)
    
class TestRTSP(unittest.TestCase):
    def setUp(self):
        """Set up test RTSP server and grabber."""
        self.port = 8554
        self.server = None
        self.grabber = None
        
    def tearDown(self):
        """Clean up resources."""
        if self.grabber:
            self.grabber.release()
        if self.server:
            self.server.stop()
    
    def test_rtsp_server_to_grabber_integration(self):
        """Test that RTSPFrameGrabber can successfully grab frames from RTSPServer."""
        # Create a static noise frame for testing
        TEST_FRAME_WIDTH = 640
        TEST_FRAME_HEIGHT = 480
        FPS = 15.0
        MOUNT_POINT = '/test'
        
        # Create RTSP server with static frame callback
        def frame_callback():
            return generate_noise_frame(TEST_FRAME_WIDTH, TEST_FRAME_HEIGHT)
        
        self.server = RTSPServer(port=self.port)
        self.server.create_stream(
            callback=frame_callback,
            width=TEST_FRAME_WIDTH,
            height=TEST_FRAME_HEIGHT,
            fps=FPS,
            mount_point=MOUNT_POINT
        )
        self.server.start()
        
        # Create RTSP grabber to connect to the server
        rtsp_url = f"rtsp://localhost:{self.port}{MOUNT_POINT}"
        config = RTSPFrameGrabberConfig(
            rtsp_url=rtsp_url,
            keep_connection_open=True,
        )
        self.grabber = RTSPFrameGrabber(config)
        
        # Grab a frame and validate it
        frame = self.grabber.grab()

        # Assertions
        self.assertIsInstance(frame, np.ndarray)

        test_frame_shape = (TEST_FRAME_HEIGHT, TEST_FRAME_WIDTH, 3)
        self.assertEqual(frame.shape, test_frame_shape)
        self.assertEqual(frame.dtype, np.uint8)

        # Verify we got a valid frame (not all zeros or corrupted)
        self.assertGreater(frame.max(), 0)
        self.assertLess(frame.min(), 255)

        # Verify that if we grab two separate frames, they are unique
        plenty_of_time_for_next_frame_to_be_available = 1 / FPS * 2
        time.sleep(plenty_of_time_for_next_frame_to_be_available)
        frame2 = self.grabber.grab()
        assert not np.array_equal(frame, frame2), "The two captured frames were not unique. Is the server publishing correct frames at a correct FPS?"


if __name__ == "__main__":
    unittest.main()
