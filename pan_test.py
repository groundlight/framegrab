from framegrab import FrameGrabber
import cv2
import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk
import os 

RTSP_URL = os.environ.get('RTSP_URL')
PAN_INCREMENT = 50

class WebcamApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Webcam Stream with Zoom and Pan")
        
        self.grabber = FrameGrabber.create_grabber(
            {
                'input_type': 'rtsp',
                'id': {
                    'rtsp_url': RTSP_URL
                }
            }
        )
        
        self.cap = self.grabber.capture
        
        # Get the original aspect ratio of the video stream
        self.original_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.original_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.aspect_ratio = self.original_width / self.original_height
        
        # Initial zoom and pan settings
        self.zoom = 1.0
        self.pan_x = 0
        self.pan_y = 0
        
        # Create canvas for video stream
        self.canvas = tk.Canvas(root, width=640, height=480)
        self.canvas.pack()
        
        # Create slider for zooming
        self.zoom_slider = ttk.Scale(root, from_=1, to=4, length=400, orient="horizontal", command=self.update_zoom)
        self.zoom_slider.pack()
        
        # Create pan buttons
        btn_frame = tk.Frame(root)
        btn_frame.pack()
        
        self.pan_left = ttk.Button(btn_frame, text="←", command=lambda: self.pan(-PAN_INCREMENT, 0))
        self.pan_left.grid(row=1, column=0)
        self.pan_left.config(state=tk.DISABLED)
        
        self.pan_up = ttk.Button(btn_frame, text="↑", command=lambda: self.pan(0, -PAN_INCREMENT))
        self.pan_up.grid(row=0, column=1)
        self.pan_up.config(state=tk.DISABLED)
        
        self.pan_down = ttk.Button(btn_frame, text="↓", command=lambda: self.pan(0, PAN_INCREMENT))
        self.pan_down.grid(row=2, column=1)
        self.pan_down.config(state=tk.DISABLED)
        
        self.pan_right = ttk.Button(btn_frame, text="→", command=lambda: self.pan(PAN_INCREMENT, 0))
        self.pan_right.grid(row=1, column=2)
        self.pan_right.config(state=tk.DISABLED)
        
        # Start the video loop
        self.update_frame()
        
    def update_zoom(self, value):
        self.zoom = float(value)
    
    def get_pan_options(self) -> list:
        
        # Get the dimensions of the grabber
        # based on dimensions of grabber and crop values, determin 
        self.grabber
        
        
        pan_options = [False, False, False, False]
        
        self.grabber.config['crop']['relative']['top']
        
        return (True, True, True, True)
        
    def update_buttons(self) -> None:
        pan_options = self.get_pan_options()
        if not pan_options[0]:  
            self.pan_left.config(state=tk.DISABLED)
        else:
            self.pan_left.config(state=tk.NORMAL)
            
        if not pan_options[1]:  
            self.pan_right.config(state=tk.DISABLED)
        else: 
            self.pan_right.config(state=tk.NORMAL)
            
        if not pan_options[2]:  
            self.pan_up.config(state=tk.DISABLED)
        else: 
            self.pan_up.config(state=tk.NORMAL)
            
        if not pan_options[3]:  
            self.pan_down.config(state=tk.DISABLED)
        else: 
            self.pan_down.config(state=tk.NORMAL)
    
    def pan(self, x, y):
        self.pan_x += x
        self.pan_y += y
        
    def pan_and_zoom_to_crop(self, pan: int, zoom: float) -> tuple:
        # Convert the pan setting (integers representing pixels) and the zoom setting (float 
        # representing zoom level) into crop values.
        new_options = {}
        self.grabber.apply_options(new_options)
        pass
        
    
    def update_frame(self):
        # print(f'{self.pan_x} | {self.pan_y} | {self.zoom}')
        frame = self.grabber.grab()

        if frame is not None:
            resized_frame = cv2.resize(frame, (640, 480), interpolation=cv2.INTER_LANCZOS4)
            
            # Convert the frame to PIL Image and display it on Tkinter canvas
            img = Image.fromarray(cv2.cvtColor(resized_frame, cv2.COLOR_BGR2RGB))
            imgtk = ImageTk.PhotoImage(image=img)
            self.canvas.create_image(0, 0, anchor=tk.NW, image=imgtk)
            self.canvas.image = imgtk
        
        # Keep looping
        self.root.after(10, self.update_frame)

    def on_closing(self):
        self.cap.release()
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = WebcamApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()
