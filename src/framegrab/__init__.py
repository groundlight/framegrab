from .cli.clitools import preview_image
from .exceptions import GrabError
from .grabber import FrameGrabber
from .motion import MotionDetector
from .rtsp_discovery import AutodiscoverMode, ONVIFDeviceInfo, RTSPDiscovery

try:
    import importlib.metadata

    # Copy the version number from pyproject.toml
    __version__ = importlib.metadata.version("framegrab")
except ModuleNotFoundError:
    # importlib.metadata was only added in py3.8
    # We're still supporting py3.7
    __version__ = "(version number only available in python 3.8+)"


__all__ = [
    "FrameGrabber",
    "MotionDetector",
    "GrabError",
    "RTSPDiscovery",
    "ONVIFDeviceInfo",
    "AutodiscoverMode",
    "preview_image",
]
