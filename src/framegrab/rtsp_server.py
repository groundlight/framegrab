import logging
import platform
import threading
import time
from typing import Callable, Dict, List

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


class Stream:
    """Represents a single RTSP stream."""
    
    def __init__(self, callback: Callable[[], np.ndarray], width: int, height: int, 
                 mount_point: str, fps: int = 30):
        self.callback = callback
        self.width = width
        self.height = height
        self.mount_point = mount_point
        self.fps = int(fps)
        self.frame_count = 0


class RTSPServer:
    """RTSP server that supports multiple streams."""

    def __init__(self, port: int = 8554):
        """Initialize RTSP server.

        Args:
            port: RTSP server port (default: 8554)
        """
        system = platform.system()
        if system == "Windows":
            raise RuntimeError(
                "RTSPServer is not supported on Windows. "
                "GStreamer RTSP server libraries are difficult to install on Windows. "
                "Please use a Linux system, WSL2, or Docker container."
            )
        elif system == "Darwin":  # macOS
            logger.warning(
                "RTSPServer has limited support on macOS. " "You may need to install GStreamer via Homebrew: "
            )

        self.port = port
        self.streams: Dict[str, Stream] = {}
        self._client_streams = {}  # Track which streams each client is accessing
        
        # GStreamer objects
        self._server = None
        self._loop = None
        self._loop_thread = None
        self._running = False

    def __str__(self) -> str:
        status = "running" if self._running else "stopped"
        stream_count = len(self.streams)
        return f"RTSPServer({status}) - port:{self.port}, streams:{stream_count}"

    def __repr__(self) -> str:
        return self.__str__()

    def create_stream(self, callback: Callable[[], np.ndarray], width: int, height: int, 
                     mount_point: str, fps: int = 30) -> None:
        """Create a new stream.
        
        Args:
            callback: Function that returns a frame when called
            width: Frame width
            height: Frame height  
            mount_point: RTSP mount point (e.g., '/stream0')
            fps: Target FPS for stream (default: 30)
        """
        if mount_point in self.streams:
            raise ValueError(f"Stream with mount point '{mount_point}' already exists")
        
        self.streams[mount_point] = Stream(callback, width, height, mount_point, fps)

    def list_streams(self) -> List[str]:
        """List all stream mount points."""
        return list(self.streams.keys())

    def list_rtsp_urls(self) -> List[str]:
        """Get a list of RTSP URLs for all streams."""
        return [f"rtsp://localhost:{self.port}{mount_point}" for mount_point in self.streams.keys()]

    def remove_stream(self, mount_point: str) -> None:
        """Remove a stream."""
        if mount_point not in self.streams:
            raise ValueError(f"Stream with mount point '{mount_point}' does not exist")
        
        del self.streams[mount_point]

    def start(self) -> None:
        """Start the RTSP server in a background thread."""
        if self._running:
            return
        
        if not self.streams:
            raise RuntimeError("No streams created. Call create_stream() first.")

        self._running = True
        self._loop_thread = threading.Thread(target=self._run_server, daemon=True)
        self._loop_thread.start()

        # Give server time to start
        time.sleep(0.5)

    def stop(self) -> None:
        """Stop the RTSP server."""
        if not self._running:
            return

        self._running = False
        if self._loop:
            self._loop.quit()
        if self._loop_thread:
            self._loop_thread.join(timeout=2.0)
        
        self.streams.clear()

    def _run_server(self) -> None:
        """Run the GStreamer RTSP server main loop."""
        Gst.init(None)

        self._server = GstRtspServer.RTSPServer()
        self._server.set_service(str(self.port))

        # Set up client connection callback
        self._server.connect("client-connected", self._on_client_connected)

        mount_points = self._server.get_mount_points()
        
        # Create a factory for each stream
        for stream in self.streams.values():
            factory = self._create_media_factory(stream)
            factory.set_shared(True)
            mount_points.add_factory(stream.mount_point, factory)

        self._server.attach(None)

        self._loop = GLib.MainLoop()
        try:
            self._loop.run()
        finally:
            self._running = False

    def _create_media_factory(self, stream: Stream):
        """Create the GStreamer media factory for a specific stream."""

        class RTSPMediaFactory(GstRtspServer.RTSPMediaFactory):
            def __init__(self, stream, rtsp_server):
                super().__init__()
                self.stream = stream
                self.rtsp_server = rtsp_server
                self.set_shared(False)

            def do_create_element(self, url):
                pipeline = (
                    f"appsrc name=source is-live=true format=GST_FORMAT_TIME "
                    f"caps=video/x-raw,format=RGB,width={self.stream.width},"
                    f"height={self.stream.height},framerate={self.stream.fps}/1 "
                    f"! videoconvert ! video/x-raw,format=I420 "
                    f"! x264enc speed-preset=ultrafast tune=zerolatency "
                    f"! rtph264pay name=pay0 pt=96"
                )
                return Gst.parse_launch(pipeline)

            def do_configure(self, rtsp_media):
                appsrc = rtsp_media.get_element().get_child_by_name("source")
                appsrc.connect("need-data", self.on_need_data)
                
                # Connect to cleanup signal to prevent resource leaks
                rtsp_media.connect("unprepared", self._on_media_unprepared)
                
                # Try to find which client is accessing this stream
                # This is a bit of a hack since GStreamer doesn't directly provide this info
                client_info = None
                for _, info in self.rtsp_server._client_streams.items():
                    if not info['streams']:  # This client hasn't accessed any streams yet
                        client_info = info
                        info['streams'].add(self.stream.mount_point)
                        break
                
                if client_info:
                    logger.info(f"RTSP Server on port {self.rtsp_server.port}: RTSP client {client_info['ip']} connected to {self.stream.mount_point}")

            def _on_media_unprepared(self, rtsp_media):
                """Clean up resources when client disconnects to prevent leaks."""
                element = rtsp_media.get_element()
                if element:
                    element.set_state(Gst.State.NULL)
                    # Force state change to complete
                    element.get_state(Gst.CLOCK_TIME_NONE)
                    # Unref to ensure complete cleanup
                    element.unref()

            def on_need_data(self, src, length):
                try:
                    frame = self.stream.callback()
                    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

                    # Convert to GStreamer buffer
                    buf = Gst.Buffer.new_allocate(None, frame.nbytes, None)
                    buf.fill(0, frame.tobytes())
                    buf.duration = Gst.SECOND // self.stream.fps
                    buf.pts = self.stream.frame_count * buf.duration
                    self.stream.frame_count += 1
                    src.emit("push-buffer", buf)
                except Exception as e:
                    logger.error(f"Error in RTSP callback for {self.stream.mount_point}: {e}")
                    # Push an empty buffer to keep the stream alive
                    buf = Gst.Buffer.new_allocate(None, 0, None)
                    src.emit("push-buffer", buf)

        return RTSPMediaFactory(stream, self)

    def _on_client_connected(self, server, client):
        """Callback when a client connects to the RTSP server."""
        connection = client.get_connection()
        client_ip = connection.get_ip()
        
        # Track this client and their streams
        self._client_streams[client] = {
            'ip': client_ip,
            'streams': set()
        }
        
        # Connect to the client's 'closed' signal to detect disconnection
        client.connect("closed", self._on_client_disconnected)

    def _on_client_disconnected(self, client):
        """Callback when a client disconnects from the RTSP server."""
        client_info = self._client_streams.pop(client, None)
        if client_info is None:
            logger.warning(f"RTSP Server on port {self.port}: Client disconnected but was not tracked")
            return
            
        streams_str = ', '.join(sorted(client_info['streams'])) if client_info['streams'] else 'no streams'
        logger.info(f"RTSP Server on port {self.port}: RTSP client {client_info['ip']} disconnected from {streams_str}")

    def __enter__(self):
        """Context manager entry."""
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.stop()
