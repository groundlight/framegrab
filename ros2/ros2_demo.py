from framegrab import FrameGrabber
import cv2

config = {
    'input_type': 'ros2',
    'name': 'My ROS2 Camera',
    'id': {
        'topic': '/groundlight/sample_image'
    },
}

grabber = FrameGrabber.create_grabber(config)
frame = grabber.grab()
print(
    'Demo started. Press "q" to quit. '
    'Press "r" to rotate the image. '
    'Press any other key to grab the next frame. '
    'Hold any other key to stream video.'
)

num_90_deg_rotations = 0

while True:
    
    if frame is None:
        print('No frame received.')
        break
    
    cv2.imshow('FrameGrab Image', frame)
    key = cv2.waitKey(0)
    if key == ord('q'):
        break
    elif key == ord('r'):
        num_90_deg_rotations += 1
        if num_90_deg_rotations > 3:
            num_90_deg_rotations = 0
            
    options = {
        'num_90_deg_rotations': num_90_deg_rotations
    }
    grabber.apply_options(options)
            
    frame = grabber.grab()
