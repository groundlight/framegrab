from framegrab import FrameGrabber
import yaml
import cv2

# load the configurations from yaml
config_path = 'sample_config.yaml'
with open(config_path, 'r') as f:
    data = yaml.safe_load(f)
    configs = yaml.safe_load(data['GL_CAMERAS'])

print('Loaded the following configurations from yaml:')
print(configs)

# Create the grabbers
grabbers = FrameGrabber.create_grabbers(configs)

while True:
    # Get a frame from each camera
    for camera_name, grabber in grabbers.items():
        frame = grabber.grab()

        cv2.imshow(camera_name, frame)
    
    key = cv2.waitKey(30)

    if key == ord('q'):
        break

cv2.destroyAllWindows()

for grabber in grabbers.values():
    grabber.release()
        