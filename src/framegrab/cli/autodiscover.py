import click
import yaml

from framegrab import FrameGrabber


@click.command()
def autodiscover():
    """Automatically Discover cameras connected to the current host (e.g. USB)."""
    # Print message to stderr
    click.echo("Discovering cameras...", err=True)

    grabbers = FrameGrabber.autodiscover()

    yaml_config = {
        "image_sources": [],
    }

    # Get a frame from each camera
    for camera_name, grabber in grabbers.items():
        frame = grabber.grab()

        click.echo(f"Grabbed frame from {camera_name} with shape {frame.shape}", err=True)
        click.echo(grabber.config, err=True)
        yaml_config["image_sources"].append(grabber.config)

        grabber.release()

    # render the yaml config dict as yaml and print it
    click.echo("Rendering sample configuration file as YAML:\n", err=True)
    print("---")
    print(f"# Auto-discovered {len(grabbers)} camera(s).")
    print(yaml.dump(yaml_config))
