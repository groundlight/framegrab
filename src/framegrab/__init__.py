from .grabber import FrameGrabber
from .motion import MotionDetector

try:
    import importlib.metadata

    # Copy the version number from pyproject.toml
    __version__ = importlib.metadata.version("framegrab")
except ModuleNotFoundError:
    # importlib.metadata was only added in py3.8
    # We're still supporting py3.7
    __version__ = "(version number available in python 3.8+)"
