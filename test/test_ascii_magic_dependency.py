import unittest

import ascii_magic
from PIL import Image


class TestAsciiMagicDependency(unittest.TestCase):
    def test_from_pillow_image_produces_ascii_art(self):
        image = Image.new("RGB", (32, 32), color="black")
        art = ascii_magic.from_pillow_image(image)

        self.assertIsInstance(art, ascii_magic.AsciiArt)

        ascii_output = art.to_ascii(columns=16)
        self.assertTrue(ascii_output.strip())
