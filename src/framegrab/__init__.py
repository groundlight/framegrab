from .cli.clitools import preview_image
from .exceptions import GrabError
from .grabber import FrameGrabber
from .motion import MotionDetector
from .rtsp_discovery import AutodiscoverMode, ONVIFDeviceInfo, RTSPDiscovery
from .unavailable_module import UnavailableModuleOrObject

try:
    import importlib.metadata

    # Copy the version number from pyproject.toml
    __version__ = importlib.metadata.version("framegrab")
except ModuleNotFoundError:
    # importlib.metadata was only added in py3.8
    # We're still supporting py3.7
    __version__ = "(version number only available in python 3.8+)"

try:
    from .ros2_client import ROS2Client
except ImportError as e:
    ROS2Client = UnavailableModuleOrObject(e)

__all__ = [
    "FrameGrabber",
    "MotionDetector",
    "GrabError",
    "RTSPDiscovery",
    "ONVIFDeviceInfo",
    "AutodiscoverMode",
    "preview_image",
    "ROS2Client",
]
