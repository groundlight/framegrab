from http.client import ImproperConnectionState
import unittest
from unittest.mock import MagicMock, patch

import numpy as np

from framegrab.grabber import RTSPFrameGrabber
import pdb
import copy

class TestBoop(unittest.TestCase):
    def test_boop(self):
        rtsp_url = "rtsp://localhost:8554/feed_0"
        rtsp_framegrabber = RTSPFrameGrabber(camera_name="rtsp_framegrabber", rtsp_url=rtsp_url, keep_connection_open=True, max_fps=10)
        di = rtsp_framegrabber.to_dict()
        di_copy = copy.deepcopy(di)
        grabber = RTSPFrameGrabber.from_dict(di)
        self.assertEqual(grabber.to_dict(), di_copy)
        frame = rtsp_framegrabber.grab()
        pdb.set_trace()
