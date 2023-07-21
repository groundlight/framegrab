from framegrab import FrameGrabber

print('Autodiscovering cameras...')

grabbers = FrameGrabber.autodiscover()

print('-' * 100)
print(f'Found {len(grabbers)} camera(s): {list(grabbers.keys())}')

# Get a frame from each camera
for camera_name, grabber in grabbers.items():
    frame = grabber.grab()
    
    print(f'Grabbed frame from {camera_name} with shape {frame.shape}')
    print(grabber.config)

    grabber.release()

print('Autodiscover demo complete.')