#!/usr/bin/env python3
"""
RTSP Publisher - grabs frames from framegrab sources and streams them via RTSP.

Usage: python rtsp_publisher.py [config.yaml]
View: vlc rtsp://localhost:8554/test
"""

import sys
import numpy as np
import cv2
import time
import gi
gi.require_version('Gst', '1.0')
gi.require_version('GstRtspServer', '1.0')
from gi.repository import Gst, GstRtspServer, GLib

from framegrab import FrameGrabber

class FrameGrabStreamer(GstRtspServer.RTSPMediaFactory):
    def __init__(self, config_path=None):
        super().__init__()
        self.frame_count = 0
        self.fps = 30
        self.width = 640
        self.height = 480
        self.config_path = config_path
        self.grabber = None
        self.setup_grabber()
    
    def setup_grabber(self):
        """Initialize the framegrab grabber."""
        try:
            # Load from YAML config file
            grabbers = FrameGrabber.from_yaml(self.config_path)
            self.grabber = grabbers[0]
            # Get first grabber if multiple sources
            if isinstance(self.grabber, dict):
                self.grabber = next(iter(self.grabber.values()))
        
            # Get sample frame to determine dimensions
            sample_frame = self.grabber.grab()
            if sample_frame is not None:
                self.height, self.width = sample_frame.shape[:2]
                print(f"Frame size: {self.width}x{self.height}")
            
        except Exception as e:
            print(f"Error setting up grabber: {e}")
            self.grabber = None
        
    def do_create_element(self, url):
        pipeline = (
            f'appsrc name=source is-live=true format=GST_FORMAT_TIME '
            f'caps=video/x-raw,format=RGB,width={self.width},height={self.height},framerate={self.fps}/1 '
            f'! videoconvert ! video/x-raw,format=I420 ! x264enc speed-preset=ultrafast tune=zerolatency '
            f'! rtph264pay name=pay0 pt=96'
        )
        return Gst.parse_launch(pipeline)
    
    def do_configure(self, rtsp_media):
        appsrc = rtsp_media.get_element().get_child_by_name('source')
        appsrc.connect('need-data', self.on_need_data)
    
    def on_need_data(self, src, length):
        try:
            if self.grabber:
                # Grab frame from framegrab source
                frame = self.grabber.grab()
            else:
                # Fallback noise frame if no grabber
                frame = np.random.randint(0, 256, (self.height, self.width, 3), dtype=np.uint8)
            
            if frame is None:
                # Create black frame if grab failed
                frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)
                cv2.putText(frame, 'No frame available', (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            
            # Resize frame if needed
            if frame.shape[:2] != (self.height, self.width):
                frame = cv2.resize(frame, (self.width, self.height))
            
            # Add frame counter
            cv2.putText(frame, f'Frame {self.frame_count}', (10, self.height - 20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # Convert BGR to RGB (OpenCV uses BGR, GStreamer expects RGB)
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            
            # Ensure frame is contiguous
            frame = np.ascontiguousarray(frame)
            
            # Convert to GStreamer buffer
            buf = Gst.Buffer.new_allocate(None, frame.nbytes, None)
            buf.fill(0, frame.tobytes())
            buf.duration = Gst.SECOND // self.fps
            buf.pts = self.frame_count * buf.duration
            
            self.frame_count += 1
            src.emit('push-buffer', buf)
            
        except Exception as e:
            print(f"Error in on_need_data: {e}")
            # Send black frame on error
            frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            frame = np.ascontiguousarray(frame)
            buf = Gst.Buffer.new_allocate(None, frame.nbytes, None)
            buf.fill(0, frame.tobytes())
            buf.duration = Gst.SECOND // self.fps
            buf.pts = self.frame_count * buf.duration
            self.frame_count += 1
            src.emit('push-buffer', buf)

def main():
    config_path = None
    if len(sys.argv) > 1:
        config_path = sys.argv[1]
        print(f"Using config: {config_path}")
    else:
        print("No config provided, using default USB camera")
    
    Gst.init(None)
    
    server = GstRtspServer.RTSPServer()
    server.set_service("8554")
    
    factory = FrameGrabStreamer(config_path)
    factory.set_shared(True)
    
    server.get_mount_points().add_factory("/test", factory)
    server.attach(None)
    
    print("RTSP server started: rtsp://localhost:8554/test")
    print("Press Ctrl+C to stop")
    
    loop = GLib.MainLoop()
    try:
        loop.run()
    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        if factory.grabber:
            factory.grabber.release()

if __name__ == "__main__":
    main()
