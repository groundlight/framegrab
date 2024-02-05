import shutil

import ascii_magic
import click
import cv2
import yaml
from imgcat import imgcat
from PIL import Image

from framegrab import FrameGrabber
from framegrab.cli.clitools import PREVIEW_COMMAND_CHOICES, preview_image

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
    type=click.Choice(PREVIEW_COMMAND_CHOICES, case_sensitive=False),
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
            frame = grabber.grab()
            if frame is None:
                print(f"Failed to grab frame from {camera_name}.")
                continue
            print(f"Grabbed frame from {camera_name}")
            preview_image(camera_name, frame, output)
    finally:
        for grabber in grabbers.values():
            try:
                grabber.release()
            except Exception as e:
                print(f"Failed to release {grabber.name}: {e}")
