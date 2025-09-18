import traceback
from pathlib import Path

import click
import yaml

from framegrab import FrameGrabber, preview_image
from framegrab.config import (
    BaslerFrameGrabberConfig,
    FileStreamFrameGrabberConfig,
    GenericUSBFrameGrabberConfig,
    HttpLiveStreamingFrameGrabberConfig,
    InputTypes,
    MockFrameGrabberConfig,
    RaspberryPiCSI2FrameGrabberConfig,
    RealSenseFrameGrabberConfig,
    ROS2GrabberConfig,
    RTSPFrameGrabberConfig,
    YouTubeLiveFrameGrabberConfig,
)

# All the input types that framegrab supports as a list of strings
INPUT_TYPES_AS_STR = [item.value for item in InputTypes]


def looks_like_yaml_file(source: str) -> bool:
    """Check if a file path has a YAML extension (.yaml or .yml)."""
    source_path = Path(source)
    return source_path.suffix.lower() in [".yaml", ".yml"]


def source_to_grabbers(source: str, input_type: str | None) -> dict[str, FrameGrabber]:
    """Create FrameGrabber objects from a source (URL, yaml file path, serial number, etc.) and optional input type."""
    # If input type is explicitly provided, use it
    if input_type:
        grabber = grabber_from_input_type(source, input_type)
        return {grabber.config.name: grabber}

    # Otherwise, try to auto-detect.
    # We can only safely auto-detect with certain grabber types. In some cases, it is ambiguous. For example, generic_usb,
    # basler and realsense all use serial number as the ID, and there doesn't seem to be a straightforward way to determine
    # grabber type from the serial number.
    if source.startswith("rtsp://"):
        rtsp_config = RTSPFrameGrabberConfig(rtsp_url=source)
        grabber = FrameGrabber.create_grabber(rtsp_config)
        return {grabber.config.name: grabber}
    elif looks_like_yaml_file(source):
        source_path = Path(source)
        if not source_path.exists():
            raise click.BadParameter(
                f"YAML config file '{source}' does not exist. " f"Please check the file path and try again."
            )
        grabbers_list = FrameGrabber.from_yaml(source)
        grabbers = {grabber.config.name: grabber for grabber in grabbers_list}
        return grabbers
    else:
        raise click.BadParameter(
            f"Unable to infer input type of SOURCE: {source}. "
            f"Try specifying the input type explicitly with -i/--input-type. Available input types are {INPUT_TYPES_AS_STR}"
        )


def grabber_from_input_type(source: str, input_type: str) -> FrameGrabber:
    """Create a FrameGrabber given an explicit input type and source identifier."""

    input_type_enum = InputTypes(input_type)
    if input_type_enum == InputTypes.GENERIC_USB:
        config = GenericUSBFrameGrabberConfig(serial_number=source)
    elif input_type_enum == InputTypes.BASLER:
        config = BaslerFrameGrabberConfig(serial_number=source)
    elif input_type_enum == InputTypes.RTSP:
        config = RTSPFrameGrabberConfig(rtsp_url=source)
    elif input_type_enum == InputTypes.REALSENSE:
        config = RealSenseFrameGrabberConfig(serial_number=source)
    elif input_type_enum == InputTypes.RPI_CSI2:
        config = RaspberryPiCSI2FrameGrabberConfig()  # No source needed
    elif input_type_enum == InputTypes.HLS:
        config = HttpLiveStreamingFrameGrabberConfig(hls_url=source)
    elif input_type_enum == InputTypes.YOUTUBE_LIVE:
        config = YouTubeLiveFrameGrabberConfig(youtube_url=source)
    elif input_type_enum == InputTypes.FILE_STREAM:
        config = FileStreamFrameGrabberConfig(filename=source)
    elif input_type_enum == InputTypes.MOCK:
        config = MockFrameGrabberConfig(serial_number=source)
    elif input_type_enum == InputTypes.ROS2:
        config = ROS2GrabberConfig(topic=source)
    else:
        raise click.BadParameter(f"Unrecognized input_type: {input_type}")

    return FrameGrabber.create_grabber(config)


@click.command()
@click.argument("source")
@click.option(
    "-i",
    "--input-type",
    type=click.Choice(INPUT_TYPES_AS_STR, case_sensitive=False),
    default=None,
    help="Specify the input type explicitly (auto-detected if not provided)",
)
@click.option(
    "-o",
    "--output",
    type=click.Choice(preview_image.OUTPUT_TYPE_CHOICES, case_sensitive=False),
    default="imgcat",
    help="Output format for preview",
)
def preview(source: str, input_type: str | None, output: str):
    """Previews images from each of the configured sources.  Must pass SOURCE: a config filename
    or an RTSP URL.
    """

    grabbers = source_to_grabbers(source, input_type)

    try:
        # Get a frame from each camera
        for camera_name, grabber in grabbers.items():
            try:
                frame = grabber.grab()
            except Exception as e:
                print(f"Error grabbing preview frame from {camera_name}: {e}")
                traceback.print_exc()
                continue
            if frame is None:
                print(f"Failed to grab preview frame from {camera_name}.")
                continue
            print(f"Grabbed preview frame from {camera_name}")
            preview_image(frame, camera_name, output)
    finally:
        for grabber in grabbers.values():
            try:
                grabber.release()
            except Exception as e:
                print(f"Failed to release {grabber.name}: {e}")
