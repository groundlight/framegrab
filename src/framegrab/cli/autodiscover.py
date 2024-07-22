import traceback

import click
import yaml
from imgcat import imgcat

from framegrab import FrameGrabber
from framegrab.cli.clitools import (
    PREVIEW_COMMAND_CHOICES,
    PREVIEW_RTSP_COMMAND_CHOICES,
    preview_image,
)
from framegrab.rtsp_discovery import AutodiscoverMode


@click.command()
@click.option(
    "--preview",
    type=click.Choice(PREVIEW_COMMAND_CHOICES, case_sensitive=False),
    default="imgcat",
    show_default=True,
)
@click.option(
    "--rtsp-discover-mode",
    type=click.Choice(PREVIEW_RTSP_COMMAND_CHOICES, case_sensitive=False),
    default="off",
    show_default=True,
)
def autodiscover(preview: str, rtsp_discover_mode: str = "off"):
    """Automatically discover cameras connected to the current host (e.g. USB)."""
    # Print message to stderr
    click.echo("Discovering cameras...", err=True)

    grabbers = FrameGrabber.autodiscover(rtsp_discover_mode=rtsp_discover_mode)

    yaml_config = {
        "image_sources": [],
    }

    # Get a frame from each camera
    for camera_name, grabber in grabbers.items():
        try:
            frame = grabber.grab()

            yaml_config["image_sources"].append(grabber.config)
            if frame is None:
                click.echo(f"Failed to grab sample frame from {camera_name}.", err=True)
                continue

            click.echo(f"Grabbed sample frame from {camera_name} with shape {frame.shape}", err=True)
            click.echo(grabber.config, err=True)
            preview_image(frame, camera_name, preview)

            grabber.release()
        except Exception:
            click.echo(f"Error while setting up {camera_name}.", err=True)
            click.echo(traceback.format_exc(), err=True)

    # render the yaml config dict as yaml and print it
    click.echo("Rendering sample configuration file as YAML:\n", err=True)
    print("---")
    print(f"# Auto-discovered {len(grabbers)} camera(s).")
    print(yaml.dump(yaml_config))
