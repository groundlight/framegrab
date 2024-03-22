#!/usr/bin/env python3
"""Reads framegrab configuration from a yaml file, creates FrameGrabber objects, and grabs images from each camera.
Remember to adjust sample_config.yaml according to your needs.
"""

from framegrab import FrameGrabber
import yaml

config_path = 'sample_config.yaml'
with open(config_path, 'r') as f:
    configs = yaml.safe_load(f)['image_sources']

print('Loaded the following configurations from yaml:')
print(configs)

grabbers = FrameGrabber.create_grabbers(configs)

for camera_name, grabber in grabbers.items():
    frame = grabber.grabimg()
    frame.show()

for grabber in grabbers.values():
    grabber.release()
        