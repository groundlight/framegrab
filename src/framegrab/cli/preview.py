import ascii_magic
import click
import cv2
from framegrab import FrameGrabber
from imgcat import imgcat
from PIL import Image
import yaml


def imgcat_preview(name: str, frame):
    """Displays the given frame in the terminal using imgcat."""
    print(f"Grabbed frame from {name}")
    imgcat(frame)


def cv2_preview(name: str, frame):
    """Displays the given frame in a cv2 window."""
    cv2.imshow(name, frame)
    key = cv2.waitKey(30)


def ascii_preview(name: str, frame):
    """Displays the given frame in the terminal using ascii art."""
    print(f"Grabbed frame from {name}")
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    pil_image = Image.fromarray(frame_rgb)
    out = ascii_magic.from_pillow_image(pil_image)
    out.to_terminal(columns=80)


def get_image_sources_from_config(config: str) -> list:
    """Returns a dictionary of image sources from the given configuration file."""
    with open(config, 'r') as f:
        configs = yaml.safe_load(f)
    if 'image_sources' not in configs:
        raise click.BadParameter('Configuration file must contain an image_sources section.')
    return configs['image_sources']

@click.command()
#@click.argument('config', type=click.Path(exists=True), help="Path to the configuration file.")
@click.argument('config', type=click.Path(exists=True))
@click.argument('output', type=click.Choice(['imgcat', 'cv2', 'ascii'], case_sensitive=False), default='imgcat')
def preview(config: str, output: str):
    """Previews images from each of the configured sources."""
    image_sources = get_image_sources_from_config(config)
    grabbers = FrameGrabber.create_grabbers(image_sources)

    preview_command = {
        'imgcat': imgcat_preview,
        'cv2': cv2_preview,
        'ascii': ascii_preview,
    }[output]

    try:
        # Get a frame from each camera
        for camera_name, grabber in grabbers.items():
            frame = grabber.grab()
            print(f"Grabbed frame from {camera_name}")
            preview_command(camera_name, frame)
    finally:
        for grabber in grabbers.values():
            try:
                grabber.release()
            except Exception as e:
                print(f"Failed to release {grabber.name}: {e}")
