#!/usr/bin/env python3
"""RTSP server command for framegrab CLI."""

import os
import signal
import subprocess
import sys
import tempfile

import click
import yaml

from framegrab import FrameGrabber
from framegrab.cli.preview import source_to_grabbers

# Only import GStreamer modules if available
try:
    import gi

    gi.require_version("Gst", "1.0")
    gi.require_version("GstRtspServer", "1.0")
    import cv2
    import numpy as np
    from gi.repository import GLib, Gst, GstRtspServer

    GSTREAMER_AVAILABLE = True
except ImportError as e:
    GSTREAMER_AVAILABLE = False
    IMPORT_ERROR = str(e)


class FrameGrabRTSPServer:
    """RTSP server that streams from FrameGrab sources."""

    def __init__(self, grabber, port=8554, mount_point="/test", fps=30):
        if not GSTREAMER_AVAILABLE:
            raise RuntimeError(f"GStreamer not available: {IMPORT_ERROR}")

        self.grabber = grabber
        self.port = port
        self.mount_point = mount_point
        self.fps = fps
        self.frame_count = 0

        # Get dimensions from first frame
        sample_frame = self.grabber.grab()
        if sample_frame is not None:
            self.height, self.width = sample_frame.shape[:2]
            click.echo(f"Detected frame size: {self.width}x{self.height}")
        else:
            self.width, self.height = 640, 480
            click.echo("Warning: Could not grab sample frame, using default 640x480")

    def start(self):
        """Start the RTSP server."""
        Gst.init(None)

        server = GstRtspServer.RTSPServer()
        server.set_service(str(self.port))

        factory = self._create_media_factory()
        factory.set_shared(True)

        mount_points = server.get_mount_points()
        mount_points.add_factory(self.mount_point, factory)

        server.attach(None)

        url = f"rtsp://localhost:{self.port}{self.mount_point}"
        click.echo(f"RTSP server started: {url}")
        click.echo("Press Ctrl+C to stop")

        # Setup signal handler for clean shutdown
        def signal_handler(sig, frame):
            click.echo("\nShutting down RTSP server...")
            self.grabber.release()
            sys.exit(0)

        signal.signal(signal.SIGINT, signal_handler)

        # Run main loop
        loop = GLib.MainLoop()
        try:
            loop.run()
        except KeyboardInterrupt:
            pass
        finally:
            self.grabber.release()

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
                try:
                    # Grab frame from framegrab source
                    frame = self.rtsp_server.grabber.grab()

                    if frame is None:
                        # Create black frame if grab failed
                        frame = np.zeros((self.rtsp_server.height, self.rtsp_server.width, 3), dtype=np.uint8)
                        cv2.putText(
                            frame, "No frame available", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2
                        )

                    # Resize frame if needed
                    if frame.shape[:2] != (self.rtsp_server.height, self.rtsp_server.width):
                        frame = cv2.resize(frame, (self.rtsp_server.width, self.rtsp_server.height))

                    # Add frame counter
                    cv2.putText(
                        frame,
                        f"Frame {self.rtsp_server.frame_count}",
                        (10, self.rtsp_server.height - 20),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0, 255, 0),
                        2,
                    )

                    # Convert BGR to RGB (OpenCV uses BGR, GStreamer expects RGB)
                    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    frame = np.ascontiguousarray(frame)

                    # Convert to GStreamer buffer
                    buf = Gst.Buffer.new_allocate(None, frame.nbytes, None)
                    buf.fill(0, frame.tobytes())
                    buf.duration = Gst.SECOND // self.rtsp_server.fps
                    buf.pts = self.rtsp_server.frame_count * buf.duration

                    self.rtsp_server.frame_count += 1
                    src.emit("push-buffer", buf)

                except Exception as e:
                    click.echo(f"Error in frame generation: {e}", err=True)

        return RTSPMediaFactory(self)


def run_docker_rtsp_server(source: str, input_type: str | None, port: int, mount_point: str, fps: int):
    """Run RTSP server using Docker container."""
    click.echo("GStreamer not available locally, using Docker container...")

    # Create temporary config file from source
    config_file = None
    config_dir = None

    try:
        # Convert source to config format
        if os.path.isfile(source):
            # Source is already a file
            config_file = os.path.abspath(source)
            config_dir = os.path.dirname(config_file)
        else:
            # Create temporary config file
            if input_type:
                config = {"image_sources": [{"input_type": input_type, "id": _parse_source_to_id(source, input_type)}]}
            else:
                # Try to auto-detect
                grabbers = source_to_grabbers(source, input_type)
                if not grabbers:
                    click.echo("Error: Could not determine input type from source", err=True)
                    return

                grabber = next(iter(grabbers.values()))
                config = {"image_sources": [grabber.config.to_framegrab_config_dict()]}
                grabber.release()

            # Write temporary config
            temp_dir = tempfile.mkdtemp()
            config_file = os.path.join(temp_dir, "temp_config.yaml")
            config_dir = temp_dir

            with open(config_file, "w") as f:
                yaml.dump(config, f)

        config_name = os.path.basename(config_file)

        # Check if Docker image exists, build if needed
        result = subprocess.run(["docker", "images", "-q", "rtsp-publisher"], capture_output=True, text=True)
        if not result.stdout.strip():
            click.echo("Building Docker image...")
            subprocess.run(
                ["docker", "build", "-t", "rtsp-publisher", "."], cwd=os.path.dirname(__file__) + "/../../.."
            )

        # Run Docker container
        docker_cmd = [
            "docker",
            "run",
            "-it",
            "--rm",
            "--privileged",
            "--name",
            "rtsp-publisher",
            "-p",
            f"{port}:8554",
            "-v",
            f"{config_dir}:/app/config:ro",
            "rtsp-publisher",
            "python3",
            "rtsp_publisher.py",
            f"/app/config/{config_name}",
        ]

        click.echo(f"Starting Docker RTSP server on port {port}")
        click.echo(f"Stream URL: rtsp://localhost:{port}/test")
        click.echo("Press Ctrl+C to stop")

        subprocess.run(docker_cmd)

    except subprocess.CalledProcessError as e:
        click.echo(f"Docker error: {e}", err=True)
    except KeyboardInterrupt:
        click.echo("\nStopping...")
    finally:
        # Clean up temporary files
        if config_file and config_file.startswith(tempfile.gettempdir()):
            try:
                os.unlink(config_file)
                os.rmdir(config_dir)
            except:
                pass


def _parse_source_to_id(source: str, input_type: str) -> dict:
    """Convert source string to framegrab ID dict based on input type."""
    if input_type == "rtsp":
        return {"rtsp_url": source}
    elif input_type == "generic_usb" or input_type == "basler" or input_type == "realsense":
        return {"serial_number": source}
    elif input_type == "file_stream":
        return {"filename": source}
    elif input_type == "hls":
        return {"hls_url": source}
    elif input_type == "youtube_live":
        return {"youtube_url": source}
    else:
        return {}


@click.command()
@click.argument("source")
@click.option(
    "-i",
    "--input-type",
    type=click.Choice(
        ["generic_usb", "rtsp", "basler", "realsense", "file_stream", "hls", "youtube_live"], case_sensitive=False
    ),
    default=None,
    help="Specify the input type explicitly (auto-detected if not provided)",
)
@click.option("-p", "--port", type=int, default=8554, help="RTSP server port (default: 8554)")
@click.option("-m", "--mount-point", default="/test", help="RTSP mount point (default: /test)")
@click.option("--fps", type=int, default=30, help="Target FPS for RTSP stream (default: 30)")
@click.option("--docker", is_flag=True, help="Force use of Docker container (auto-detected if GStreamer unavailable)")
def rtsp_server(source: str, input_type: str | None, port: int, mount_point: str, fps: int, docker: bool):
    """Start an RTSP server streaming from a framegrab source.

    SOURCE can be an RTSP URL, config file, serial number, etc.

    The command automatically uses Docker if GStreamer is not available locally.

    Examples:
        framegrab rtsp-server camera_config.yaml
        framegrab rtsp-server rtsp://camera-ip/stream -p 8555
        framegrab rtsp-server 12345 -i generic_usb --fps 15
        framegrab rtsp-server config.yaml --docker
    """

    # Use Docker if explicitly requested or if GStreamer not available
    if docker or not GSTREAMER_AVAILABLE:
        run_docker_rtsp_server(source, input_type, port, mount_point, fps)
        return

    # Use native GStreamer implementation
    try:
        grabbers = source_to_grabbers(source, input_type)

        if not grabbers:
            click.echo("Error: No grabbers found for the provided source", err=True)
            sys.exit(1)

        # Use first grabber if multiple sources
        grabber = next(iter(grabbers.values()))

        click.echo(f"Starting native RTSP server for: {grabber.config}")

        # Create and start RTSP server
        rtsp_server_instance = FrameGrabRTSPServer(grabber, port=port, mount_point=mount_point, fps=fps)
        rtsp_server_instance.start()

    except Exception as e:
        click.echo(f"Error starting RTSP server: {e}", err=True)
        sys.exit(1)
