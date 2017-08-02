import picamera
import picamera.array
import cv2
import numpy as np
import math
from image_processing import grayscale,canny,gaussian_blur,sobel,\
     region_of_interest, Counters_lane, draw_lines,\
     hough_lines_p,threshold, Counters_signal, Counters_Pheeno
from capture_image import image_capture


def detect_red(image):


    # Converting to HSV
    hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    # Masking red color
    mask_red = cv2.inRange(hsv,red_bound_lower,red_bound_upper)
    mask_red_image = cv2.bitwise_and(image,image,mask=mask_red)

    #Gaussian Filter
    kernel_size = 7
    gauss_gray_red = gaussian_blur(mask_red_image,kernel_size)

    imshape = image.shape
    lower_left = [imshape[1]/100,0]
    lower_right = [imshape[1]-imshape[1]/100,0]
    top_left = [imshape[1]/5-imshape[1]/1,130]
    top_right = [imshape[1]/5+imshape[1]/1,130]
    vertices = [np.array([lower_left,top_left,top_right,lower_right],dtype=np.int32)]
    roi_image = region_of_interest(gauss_gray_red, vertices)

    # Canny canny_edges

    low_threshold = 100
    high_threshold = 200
    canny_edges_red = canny(roi_image,low_threshold,high_threshold)

    red_points = Counters_signal(canny_edges_red,image)
    #cv2.imshow('Red', gauss_gray_red)

    if red_points != None:
        cv2.rectangle(image,(red_points[0],red_points[1]),((red_points[0]+red_points[2]),(red_points[1]+red_points[3])),(255,0,0),1)
        #cv2.circle(image, (int(red_points[0]),int(red_points[1])),int(red_points[2]),(255,0,0),2)
        cv2.putText(image,'Red',((int(red_points[0])+20),(int(red_points[1])+20)),2,1,(255,255,255))
        return 'RED'



def detect_green(image):


    # Converting to HSV
    hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    # Masking red color
    mask_green = cv2.inRange(hsv,green_bound_lower,green_bound_upper)
    mask_green_image = cv2.bitwise_and(image,image,mask=mask_green)

    #Gaussian Filter
    kernel_size = 7
    gauss_gray_green = gaussian_blur(mask_green_image,kernel_size)

    imshape = image.shape
    lower_left = [imshape[1]/100,0]
    lower_right = [imshape[1]-imshape[1]/100,0]
    top_left = [imshape[1]/5-imshape[1]/1,130]
    top_right = [imshape[1]/5+imshape[1]/1,130]
    vertices = [np.array([lower_left,top_left,top_right,lower_right],dtype=np.int32)]
    roi_image = region_of_interest(gauss_gray_green, vertices)


    # Canny canny_edges

    low_threshold = 100
    high_threshold = 200
    canny_edges_green = canny(roi_image,low_threshold,high_threshold)

    # Find the contours and return points
    green_points = Counters_signal(canny_edges_green,image)
    #cv2.imshow('Video green', roi_image)
    #print green_points


    if green_points != None:
        cv2.rectangle(image,(green_points[0],green_points[1]),((green_points[0]+green_points[2]),(green_points[1]+green_points[3])),(255,0,0),1)
        #cv2.circle(image, (int(green_points[0]),int(green_points[1])),int(green_points[2]),(255,0,0),2)
        cv2.putText(image,'GREEN',((int(green_points[0])+20),(int(green_points[1])+20)),2,1,(255,255,255))
        return 'GREEN'



if __name__ == '__main__':

    mode = 'pi_camera'

    if mode == 'pi_camera':

        while True:
            # Get the input image
            frame = image_capture()

            # Start the lane detection process
            lights_detect = Lane_tracking(frame, display = True)

            #Display the detected lane on the frame
            #cv2.imshow('Input Image', blend)

            # Break if cv2 window is open by pressig "q" button.
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
