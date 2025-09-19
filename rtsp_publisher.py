#!/usr/bin/env python3
"""
RTSP Publisher - creates noise frames and streams them via RTSP.

SYSTEM REQUIREMENTS (no way around this for true RTSP):
sudo apt-get install python3-gi gir1.2-gst-rtsp-server-1.0 gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-ugly

Usage: python rtsp_publisher.py
View: vlc rtsp://localhost:8554/test
"""

import numpy as np
import cv2
import time
import gi
gi.require_version('Gst', '1.0')
gi.require_version('GstRtspServer', '1.0')
from gi.repository import Gst, GstRtspServer, GLib

class NoiseStreamer(GstRtspServer.RTSPMediaFactory):
    def __init__(self):
        super().__init__()
        self.frame_count = 0
        self.fps = 30
        self.width = 640
        self.height = 480
        
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
        # Create noise frame
        frame = np.random.randint(0, 256, (self.height, self.width, 3), dtype=np.uint8)
        
        # Add frame counter
        cv2.putText(frame, f'Frame {self.frame_count}', (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        
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

def main():
    Gst.init(None)
    
    server = GstRtspServer.RTSPServer()
    server.set_service("8554")
    
    factory = NoiseStreamer()
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

if __name__ == "__main__":
    main()
