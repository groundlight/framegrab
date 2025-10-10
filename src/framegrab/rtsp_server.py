import logging
import platform
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Tuple

import cv2
import numpy as np

from .unavailable_module import UnavailableModuleOrObject

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


@dataclass
class ClientEntry:
    """Per-client state for a single RTSP consumer.

    Holds the per-connection `appsrc` element and a monotonically
    increasing `frame_count` used to compute buffer PTS/duration.
    """

    appsrc: Any
    frame_count: int = 0


@dataclass
class MountState:
    """Mutable runtime state for a single RTSP mount point.

    - `clients`: active consumers attached to this mount
    - `clients_lock`: guards access to `clients` and `producer`
    - `producer`: (thread, stop_event) driving frames for this mount, or None
    """

    clients: List[ClientEntry] = field(default_factory=list)
    clients_lock: threading.Lock = field(default_factory=threading.Lock)
    # (thread, stop_event) when a producer is running, else None
    producer: Optional[Tuple[threading.Thread, threading.Event]] = None


class Stream:
    def __init__(self, callback: Callable[[], np.ndarray], width: int, height: int, mount_point: str, fps: float):
        self.callback = callback
        self.width = width
        self.height = height
        self.mount_point = mount_point
        self.fps = fps


class RTSPServer:
    def __init__(self, port: int = 8554):
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

        self.port = int(port)
        self.streams: Dict[str, Stream] = {}

        # mount_point -> MountState
        self._mounts: Dict[str, MountState] = {}
        self._server = None
        self._loop = None
        self._loop_thread = None
        self._running = False

    def create_stream(self, callback: Callable[[], np.ndarray], width: int, height: int, mount_point: str, fps: float):
        if self._running:
            raise RuntimeError(
                "RTSPServer has already started. Streams can only be created prior to starting the server."
            )

        if mount_point in self.streams:
            raise ValueError(f"Stream '{mount_point}' exists")
        self.streams[mount_point] = Stream(callback, width, height, mount_point, fps)
        self._mounts[mount_point] = MountState()

    def list_rtsp_urls(self) -> List[str]:
        return [f"rtsp://localhost:{self.port}{m}" for m in self.streams.keys()]

    def start(self):
        if self._running:
            raise RuntimeError("RTSPServer is already running.")

        if not self.streams:
            raise RuntimeError("No streams created. Please call `create_stream` first.")

        self._running = True
        self._loop_thread = threading.Thread(target=self._run_server, daemon=True)
        self._loop_thread.start()
        time.sleep(0.2)

    def stop(self):
        if not self._running:
            return
        self._running = False
        # stop producers
        for mount in list(self._mounts.values()):
            prod = mount.producer
            if prod:
                thr, stop_evt = prod
                stop_evt.set()
                thr.join(timeout=1.0)
                mount.producer = None
        if self._loop:
            self._loop.quit()
        if self._loop_thread:
            self._loop_thread.join(timeout=2.0)
        self.streams.clear()
        self._mounts.clear()

    def _run_server(self):
        Gst.init(None)
        self._server = GstRtspServer.RTSPServer()
        self._server.set_service(str(self.port))
        self._server.connect("client-connected", self._on_client_connected)
        mount_points = self._server.get_mount_points()
        for s in self.streams.values():
            f = self._create_media_factory(s)
            f.set_shared(False)  # per-client pipelines
            mount_points.add_factory(s.mount_point, f)
        self._server.attach(None)
        self._loop = GLib.MainLoop()
        try:
            self._loop.run()
        finally:
            self._running = False

    def _create_media_factory(self, stream: Stream):
        class Factory(GstRtspServer.RTSPMediaFactory):
            def __init__(self, stream, server):
                super().__init__()
                self.stream = stream
                self.server = server

            def do_create_element(self, url):
                fps_int = int(round(self.stream.fps, 0))  # Gstreamer wants an int here
                pipeline = (
                    f"appsrc name=source is-live=true format=GST_FORMAT_TIME "
                    f"caps=video/x-raw,format=RGB,width={self.stream.width},"
                    f"height={self.stream.height},framerate={fps_int}/1 "
                    f"! videoconvert ! video/x-raw,format=I420 "
                    f"! x264enc speed-preset=ultrafast tune=zerolatency "
                    f"! rtph264pay name=pay0 pt=96"
                )
                return Gst.parse_launch(pipeline)

            def do_configure(self, rtsp_media):
                appsrc = rtsp_media.get_element().get_child_by_name("source")
                appsrc.set_property("is-live", True)
                appsrc.set_property("format", Gst.Format.TIME)
                appsrc.set_property("do-timestamp", False)  # we manage per-client PTS

                mount = self.server._mounts[self.stream.mount_point]
                duration = int(Gst.SECOND / float(self.stream.fps))

                client = ClientEntry(appsrc=appsrc)
                with mount.clients_lock:
                    mount.clients.append(client)

                # start producer if absent
                if mount.producer is None:
                    stop_evt = threading.Event()

                    def producer():
                        period = 1.0 / float(self.stream.fps)
                        next_t = time.monotonic()
                        while not stop_evt.is_set():
                            now = time.monotonic()
                            s = next_t - now
                            if s > 0:
                                time.sleep(s)
                            next_t += period

                            try:
                                frame = self.stream.callback()
                                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                                fb = frame.tobytes()
                            except Exception:
                                logger.exception("grab failed for %s", self.stream.mount_point)
                                fb = b""

                            with mount.clients_lock:
                                clients = list(mount.clients)

                            for c in clients:
                                GLib.idle_add(push_to_client, c, fb, duration)

                            with mount.clients_lock:
                                if not mount.clients:
                                    break

                    thr = threading.Thread(target=producer, daemon=True, name=f"prod-{self.stream.mount_point}")
                    mount.producer = (thr, stop_evt)
                    thr.start()

                def remove_client():
                    with mount.clients_lock:
                        mount.clients = [c for c in mount.clients if c is not client]
                        if not mount.clients and mount.producer:
                            thr, stop_evt = mount.producer
                            stop_evt.set()
                            thr.join(timeout=1.0)
                            mount.producer = None

                # cleanup when media is unprepared
                rtsp_media.connect("unprepared", lambda *a: remove_client())

        def push_to_client(client_entry, frame_bytes: bytes, duration_ns: int):
            app = client_entry.appsrc
            if app is None:
                return

            try:
                buf = Gst.Buffer.new_allocate(None, len(frame_bytes), None)
                buf.fill(0, frame_bytes)
                fc = client_entry.frame_count
                buf.pts = int(fc * duration_ns)
                buf.duration = int(duration_ns)
                client_entry.frame_count = fc + 1
                app.emit("push-buffer", buf)
            except Exception:
                logger.exception("push failed; removing client")
                # remove failing client
                for mount in self._mounts.values():
                    with mount.clients_lock:
                        if client_entry in mount.clients:
                            mount.clients.remove(client_entry)
                            if not mount.clients and mount.producer:
                                thr, stop_evt = mount.producer
                                stop_evt.set()
                                thr.join(timeout=1.0)
                                mount.producer = None
                            break

        return Factory(stream, self)

    def _on_client_connected(self, server, client):
        conn = client.get_connection()
        ip = conn.get_ip()
        logger.info(f"RTSP client connected: ip={ip}")

        client.connect("closed", self._on_client_disconnected)

    def _on_client_disconnected(self, client):
        conn = client.get_connection()
        ip = conn.get_ip()
        logger.info(f"RTSP client disconnected: ip={ip}")

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()
