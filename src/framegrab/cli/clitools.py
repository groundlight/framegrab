import shutil

import ascii_magic
import cv2
from imgcat import imgcat
from PIL import Image


def imgcat_preview(name: str, frame):
    """Displays the given frame in the terminal using imgcat.
    This requires an advanced terminal like iTerm2."""
    print(f"Previewing image from camera {name} in terminal. This requires an advanced terminal like iTerm2.")
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    imgcat(frame_rgb)


def cv2_preview(name: str, frame):
    """Displays the given frame in a cv2 window, and wait for a key."""
    cv2.imshow(name, frame)
    print(f"Previewing image in cv2 window. Select the window and press any key to continue.")
    _ = cv2.waitKey(0)
    cv2.destroyAllWindows()


def ascii_preview(name: str, frame):
    """Displays the given frame in the terminal using ascii art."""
    columns, _ = shutil.get_terminal_size()
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    pil_image = Image.fromarray(frame_rgb)
    out = ascii_magic.from_pillow_image(pil_image)
    out.to_terminal(columns=columns)


def null_preview(name: str, frame):
    """Does nothing."""
    pass


_PREVIEW_COMMANDS = {
    "imgcat": imgcat_preview,
    "cv2": cv2_preview,
    "ascii": ascii_preview,
    "none": null_preview,
}

PREVIEW_COMMAND_CHOICES = list(_PREVIEW_COMMANDS.keys())


def preview_image(frame, title: str, output_type: str):
    """Displays the given frame using the given output method.

    :param frame: The frame to preview.
    :param title: A string title to display with the preview.
    :param output_type: The method to use for previewing the frame.  Valid choices for `output_type` can be found in `preview_image.OUTPUT_TYPE_CHOICES`.
    """
    if output_type not in PREVIEW_COMMAND_CHOICES:
        raise ValueError(f"Invalid output method: {output_type}.  Valid options are {PREVIEW_COMMAND_CHOICES}.")
    command = _PREVIEW_COMMANDS[output_type]
    if frame is None:
        print(f"Trying to preview None frame from {title}.")
        return
    command(title, frame)


preview_image.OUTPUT_TYPE_CHOICES = PREVIEW_COMMAND_CHOICES
