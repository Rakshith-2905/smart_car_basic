import picamera
import picamera.array
import cv2
import time
import numpy as np

camera = picamera.PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
raw_capture = picamera.array.PiRGBArray(camera, size=(640, 480))
time.sleep(0.1)

def image_capture(frame_rate = 32 , image_resolution = (640, 480) ,  verbose = False):
    """
    Capture the image from the pi camera.
    :param frame_rate: frame rate of the pi camera
    :param image_resolution: the rasolution of the captured image
    :param verbose: if True, show the transformation result
    :return: return a numpy array of the image 
    """

    for frame in camera.capture_continuous(raw_capture, format="bgr", use_video_port=True):
        
        image = raw_capture.array
        
        # clear the stream in preparation for the next frame
        raw_capture.truncate(0)

        # stream.array contains the image array in bgr order!
        return image



if __name__ == '__main__':

    while True:
        # Get the input image
        image = image_capture()

        cv2.imshow('Input Image', image)

        # Break if cv2 window is open by pressig "q" button.
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
