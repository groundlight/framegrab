import shutil
import traceback

import ascii_magic
import click
import cv2
import yaml
from imgcat import imgcat
from PIL import Image

from framegrab import FrameGrabber, preview_image


def get_image_sources_from_config(config: str) -> list:
    """Returns a dictionary of image sources from the given configuration file."""
    with open(config, "r") as f:
        configs = yaml.safe_load(f)
    if "image_sources" not in configs:
        raise click.BadParameter("Configuration file must contain an image_sources section.")
    return configs["image_sources"]


@click.command()
@click.argument("config", type=click.Path(exists=True))
@click.argument(
    "output",
    type=click.Choice(preview_image.OUTPUT_TYPE_CHOICES, case_sensitive=False),
    default="imgcat",
)
def preview(config: str, output: str):
    """Previews images from each of the configured sources.  Must pass CONFIG: a filename with
    a configuration.
    """
    image_sources = get_image_sources_from_config(config)
    grabbers = FrameGrabber.create_grabbers(image_sources)

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
