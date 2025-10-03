import logging
import threading
import time
from typing import Callable

import cv2
import numpy as np

from .unavailable_module import UnavailableModuleOrObject

# Only import GStreamer modules if available
try:
    import gi

    gi.require_version("Gst", "1.0")
    gi.require_version("GstRtspServer", "1.0")
    from gi.repository import GLib, Gst, GstRtspServer
except ImportError as e:
    gi = UnavailableModuleOrObject(e)
    GLib = UnavailableModuleOrObject(e)
    Gst = UnavailableModuleOrObject(e)
    GstRtspServer = UnavailableModuleOrObject(e)

logger = logging.getLogger(__name__)


class RTSPServer:
    """Simple RTSP server that streams frames via callback."""

    def __init__(
        self,
        callback: Callable[[], np.ndarray],
        width: int,
        height: int,
        port: int = 8554,
        mount_point: str = "/stream",
        fps: int = 30,
    ):
        """Initialize RTSP server.

        Args:
            callback: Function that returns a frame when called
            width: Frame width (required)
            height: Frame height (required)
            port: RTSP server port (default: 8554)
            mount_point: RTSP mount point (default: /stream)
            fps: Target FPS for RTSP stream (default: 30)
        """
        # This will raise the ImportError if GStreamer is not available
        # following the UnavailableModuleOrObject pattern
        _ = gi, cv2, GLib, Gst, GstRtspServer

        self.callback = callback
        self.port = port
        self.mount_point = mount_point
        self.fps = fps
        self.width = width
        self.height = height

        self.frame_count = 0

        # GStreamer objects
        self._server = None
        self._loop = None
        self._loop_thread = None
        self._running = False

        self.rtsp_url = f"rtsp://localhost:{self.port}{self.mount_point}"

    def __str__(self) -> str:
        status = "running" if self._running else "stopped"
        return f"RTSPServer({status}) - {self.rtsp_url}"

    def __repr__(self) -> str:
        return (
            f"RTSPServer(port={self.port}, mount_point='{self.mount_point}', "
            f"fps={self.fps}, width={self.width}, height={self.height}, "
            f"running={self._running})"
        )

    def start(self) -> None:
        """Start the RTSP server in a background thread."""
        if self._running:
            return

        self._running = True
        self._loop_thread = threading.Thread(target=self._run_server, daemon=True)
        self._loop_thread.start()

        # Give server time to start
        time.sleep(0.5)

        logger.info(f"RTSP server started: {self.rtsp_url}")

    def stop(self) -> None:
        """Stop the RTSP server."""
        if not self._running:
            return

        self._running = False
        if self._loop:
            self._loop.quit()
        if self._loop_thread:
            self._loop_thread.join(timeout=2.0)

    def _run_server(self) -> None:
        """Run the GStreamer RTSP server main loop."""
        Gst.init(None)

        self._server = GstRtspServer.RTSPServer()
        self._server.set_service(str(self.port))

        factory = self._create_media_factory()
        factory.set_shared(True)

        mount_points = self._server.get_mount_points()
        mount_points.add_factory(self.mount_point, factory)

        self._server.attach(None)

        self._loop = GLib.MainLoop()
        try:
            self._loop.run()
        finally:
            self._running = False

    def _create_media_factory(self):
        """Create the GStreamer media factory."""

        class RTSPMediaFactory(GstRtspServer.RTSPMediaFactory):
            def __init__(self, rtsp_server):
                super().__init__()
                self.rtsp_server = rtsp_server

            def do_create_element(self, url):
                pipeline = (
                    f"appsrc name=source is-live=true format=GST_FORMAT_TIME "
                    f"caps=video/x-raw,format=RGB,width={self.rtsp_server.width},"
                    f"height={self.rtsp_server.height},framerate={self.rtsp_server.fps}/1 "
                    f"! videoconvert ! video/x-raw,format=I420 "
                    f"! x264enc speed-preset=ultrafast tune=zerolatency "
                    f"! rtph264pay name=pay0 pt=96"
                )
                return Gst.parse_launch(pipeline)

            def do_configure(self, rtsp_media):
                appsrc = rtsp_media.get_element().get_child_by_name("source")
                appsrc.connect("need-data", self.on_need_data)

            def on_need_data(self, src, length):
                # Call the user's callback to get the latest frame
                frame = self.rtsp_server.callback()
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

                # Convert to GStreamer buffer
                buf = Gst.Buffer.new_allocate(None, frame.nbytes, None)
                buf.fill(0, frame.tobytes())
                buf.duration = Gst.SECOND // self.rtsp_server.fps
                buf.pts = self.rtsp_server.frame_count * buf.duration

                self.rtsp_server.frame_count += 1
                src.emit("push-buffer", buf)

        return RTSPMediaFactory(self)

    def __enter__(self):
        """Context manager entry."""
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.stop()
