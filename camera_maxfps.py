import picamera
import signal
import numpy as np

stop_process = False

def signal_handler(signal, frame):
        global stop_process
        stop_process = True
signal.signal(signal.SIGINT, signal_handler)

RES = (640, 480)
FPS = 30

# Class to process camera messages
class Stream():
    
    def __init__(self):
      # Compute map1 and map2 for undistortion
      pass
      
    def undistort(self, I):
      pass

    # Called when new image is available
    def write(self, data):
        
        # Get the Y component which is the gray image
        data_y = data[:RES[0]*RES[1]]
        I_distorted = np.array(data_y).reshape(RES)
        
        I = self.undistort(I_distorted)
        
        
if __name__ == "__main__":

    # Start capturing camera images
    with picamera.PiCamera() as camera:
        
        camera.resolution = RES
        camera.framerate = FPS
        
        try:
            camera.start_recording(Stream(), format='yuv')
            while not stop_process:
                camera.wait_recording(1)
        except:
            pass
        
        #camera.stop_recording()
        camera.close()
