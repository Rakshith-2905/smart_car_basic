import picamera
import picamera.array
import cv2
import numpy as np
import serial
import os.path
import sys
import socket
import time
import math

global count
count =0
stop = time.time()
from image_processing import grayscale,canny,gaussian_blur,sobel,\
     region_of_interest, Counters_lane, draw_lines,\
     hough_lines_p,threshold, Counters_signal, Counters_Pheeno

from math_processing import slope,average_of_lines


BAUD_RATE = 9600

prev_error=0



prev_color ='None'
color2_p ='None'


red_bound_upper = np.array([180, 255, 255], dtype="uint8")
red_bound_lower = np.array([122, 143, 160], dtype="uint8")
green_bound_upper = np.array([180, 255, 255], dtype="uint8")
green_bound_lower = np.array([65, 255, 160], dtype="uint8")


white_bound_upper = np.array([180, 255, 255], dtype="uint8")
white_bound_lower = np.array([72, 0, 146], dtype="uint8")
yellow_bound_upper = np.array([180, 255, 255], dtype="uint8")
yellow_bound_lower = np.array([0, 120, 148], dtype="uint8")
lane_bound_upper = np.array([180, 255, 255], dtype="uint8")
lane_bound_lower = np.array([0, 0, 170], dtype="uint8")


# USB Serial Options
usb_options = ['/dev/ttyACM0', '/dev/ttyACM1']
usb_options_existing = filter(os.path.exists, usb_options)
assert(len(usb_options_existing) == 1)
usb_port = usb_options_existing[0]
print(usb_port)

while True:
    try:
        print("Attempting to connect...")
        usb = serial.Serial(usb_port, BAUD_RATE)
        usb.close()
        time.sleep(2)
        usb.open()
        time.sleep(2)
        print("Connected!")
        break
    except serial.SerialException:
        print("[ERROR] Could not connect to requested port.")
        time.sleep(1)





def detect_red(image):
    

    # Converting to HSV
    hsv = cv2.cvtColor(stream.array,cv2.COLOR_BGR2HSV)
    # Masking red color
    mask_red = cv2.inRange(hsv,red_bound_lower,red_bound_upper)
    mask_red_image = cv2.bitwise_and(stream.array,stream.array,mask=mask_red)

    #Gaussian Filter
    kernel_size = 7
    gauss_gray_red = gaussian_blur(mask_red_image,kernel_size)

    imshape = stream.array.shape
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
                                     
    # Find the contours and return points
    red_points = Counters_signal(canny_edges_red,stream.array)
    #cv2.imshow('Red', gauss_gray_red)
  
    if red_points != None:
        cv2.rectangle(stream.array,(red_points[0],red_points[1]),((red_points[0]+red_points[2]),(red_points[1]+red_points[3])),(255,0,0),1)
        #cv2.circle(stream.array, (int(red_points[0]),int(red_points[1])),int(red_points[2]),(255,0,0),2)
        cv2.putText(stream.array,'Red',((int(red_points[0])+20),(int(red_points[1])+20)),2,1,(255,255,255))
        return 'RED'



def detect_green(image):
    

    # Converting to HSV
    hsv = cv2.cvtColor(stream.array,cv2.COLOR_BGR2HSV)
    # Masking red color
    mask_green = cv2.inRange(hsv,green_bound_lower,green_bound_upper)
    mask_green_image = cv2.bitwise_and(stream.array,stream.array,mask=mask_green)

    #Gaussian Filter
    kernel_size = 7
    gauss_gray_green = gaussian_blur(mask_green_image,kernel_size)
    
    imshape = stream.array.shape
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
    green_points = Counters_signal(canny_edges_green,stream.array)
    #cv2.imshow('Video green', roi_image)
    #print green_points

    
    if green_points != None:
        cv2.rectangle(stream.array,(green_points[0],green_points[1]),((green_points[0]+green_points[2]),(green_points[1]+green_points[3])),(255,0,0),1)
        #cv2.circle(stream.array, (int(green_points[0]),int(green_points[1])),int(green_points[2]),(255,0,0),2)
        cv2.putText(stream.array,'GREEN',((int(green_points[0])+20),(int(green_points[1])+20)),2,1,(255,255,255))
        return 'GREEN'



def Lane_tracking(image):
    start = time.time() 
    global prev_error
    a=0.20

    # Find HSV Values, if needed. r and c values calibrated
            # for 640x480.
            # r1 = 300
            # r2 = 340
            # c1 = 200

            # c2 = 240
            # roi = stream.array[r1:r2, c1:c2]
            # hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    # Converting to HSV
    hsv = cv2.cvtColor(stream.array,cv2.COLOR_BGR2HSV)
    # Masking red color
    mask_lane = cv2.inRange(hsv,lane_bound_lower,lane_bound_upper)
    mask_lane_image = cv2.bitwise_and(stream.array,stream.array,mask=mask_lane)

    # Converting to grayscale
    gray = grayscale(mask_lane_image)

    #Applying Gaussian Blur
    gaussian = gaussian_blur(gray , 5)

    #thresholding
    #thresh = threshold(gaussian)

    imshape = stream.array.shape
    lower_left = [imshape[1]/100,imshape[0]]
    lower_right = [imshape[1]-imshape[1]/100,imshape[0]]
    top_left = [imshape[1]/2-imshape[1]/1,imshape[0]/2+imshape[0]/10]
    top_right = [imshape[1]/2+imshape[1]/1,imshape[0]/2+imshape[0]/10]
    vertices = [np.array([lower_left,top_left,top_right,lower_right],dtype=np.int32)]
    roi_image = region_of_interest(gaussian, vertices)



    # Canny canny_edges

    low_threshold = 100
    high_threshold = 200
    canny_edges = canny(roi_image,low_threshold,high_threshold)
    #cv2.imshow('Video1', roi_image)


##    """Hough Lines transforms"""

    rho = 1
    theta = np.pi/180
    #threshold is minimum number of intersections in a grid for candidate line to go to output
    threshold = 20

    min_line_len = 1
    max_line_gap = 20
    line_hough = hough_lines_p(canny_edges,stream.array, rho, theta, threshold,min_line_len,max_line_gap)
    line_image = line_hough[0]
    error = line_hough[1]
    if(error !=None):
        #print error
        error = error*(a)+ prev_error*(a-1)
        prev_error = error
##        cv2.imshow('Video1', line_image)
        print time.time()-start
        return (error)






def detect_pheeno(image):
    # Converting to HSV
     
    hsv = cv2.cvtColor(stream.array,cv2.COLOR_BGR2HSV)

    

    # Masking red color
    mask_green = cv2.inRange(hsv,red_bound_lower,red_bound_upper)
    mask_green_image = cv2.bitwise_and(stream.array,stream.array,mask=mask_green)


    
    #Gaussian Filter
    kernel_size = 9
    gauss_gray_green = gaussian_blur(mask_green_image,kernel_size)


    
    imshape = stream.array.shape
    lower_left = [imshape[1]/100,140]
    lower_right = [imshape[1]-imshape[1]/100,140]
    top_left = [imshape[1]/5-imshape[1]/1,imshape[0]-140]
    top_right = [imshape[1]/5+imshape[1]/1,imshape[0]-140]
    vertices = [np.array([lower_left,top_left,top_right,lower_right],dtype=np.int32)]
    roi_image = region_of_interest(gauss_gray_green, vertices)

    threshold_image = threshold(roi_image)

        # Canny canny_edges

    low_threshold = 100
    high_threshold = 200
    canny_edges = canny(threshold_image,low_threshold,high_threshold)
    


        # Find the contours and return points
    green_points_pheeno = Counters_Pheeno(canny_edges)
    #cv2.imshow('Video green', gauss_gray_green)
    #cv2.imshow('Video1', roi_image)
    

    if green_points_pheeno != None:
        #cv2.rectangle(stream.array,(green_points_pheeno[0],green_points_pheeno[1]),((green_points_pheeno[0]+green_points_pheeno[2]),(green_points_pheeno[1]+green_points_pheeno[3])),(255,0,0),1)
        #cv2.circle(stream.array, (int(green_points_pheeno[0]),int(green_points_pheeno[1])),int(green_points_pheeno[2]),(255,0,0),2)
        #cv2.putText(stream.array,'PHEENO',((int(green_points_pheeno[0])+20),(int(green_points_pheeno[1])+20)),2,1,(255,255,255))
        #cv2.imshow('Video', roi_image)
        return True
    

# Camera Routine
with picamera.PiCamera() as camera:
    with picamera.array.PiRGBArray(camera) as stream:
        camera.resolution = (640, 480)
        # camera.resolution = (320, 240)
        
        print stop

        try:
            while True:
                
                
                camera.capture(stream, 'bgr', use_video_port=True)
                #print time.time()-stop

                # stream.array contains the image array in bgr order!

                #Do the required image processing
                #Signal Detection Function call
                pheeno = detect_pheeno(stream.array)



                color2 = detect_green(stream.array)
                color1 = detect_red(stream.array)


                  
                if color2 == 'GREEN' and prev_color != 'GREEN' and pheeno != True:
                    #Lane Tracking Function call
                    error = Lane_tracking(stream.array)
                    #print error, "Green"
                
                    if error != None:
                        
                        error_string = 'GREEN'+';'+str(int(error))+':'
                        print(error_string)
                        message1 = error_string
                        # Send information.
                        usb.flush()
                        usb.write(message1)          
                        usb.flush()
                        error_string = 'None'
                        color2_p='None'
                        prev_color = 'GREEN'

                if color1 == 'RED':
                    error = 1000
                    error_string = 'RED'+';'+str(int(error))+':'
                    
                    print(error_string)
                    message2 = error_string
                    # Send information.
                    usb.flush()
                    usb.write(message2)          
                    usb.flush()
                    error_string = 'None'
                    #color1='None'
                    color1_p='None'
                    prev_color = 'RED'

                if color2_p == 'None' and prev_color != 'RED' and pheeno != True:
                    #Lane Tracking Function call
                    error = Lane_tracking(stream.array)
                    
                
                    if error:
                        
                        error_string = 'None'+';'+str(int(error))+':'
                        print(error_string)
                        message1 = error_string
                        # Send information.
                        usb.flush()
                        usb.write(message1)          
                        usb.flush()
                        error_string = 'None'
                        prev_color = 'GREEN'
                    else:
                        error_string = 'None'+';'+'None'+':'
                        print(error_string)
                        message3 = error_string
                        # Send information.
                        usb.flush()
                        usb.write(message3)          
                        usb.flush()
                        error_string = 'None'
##                       
                if prev_color=='RED' and color1==None and color2 ==None and pheeno != True:
                    start_time = time.time()
                    elap_time=0
                    
                    #while elap_time<0.200:
                    error_string = 'REVERSE'+';'+'None'+':'
                    print(error_string)
                    message4 = error_string
                    # Send information.
                    usb.flush()
                    usb.write(message4)          
                    usb.flush()
                    error_string = 'None'
                    elap_time = time.time() - start_time
                    
                    print elap_time

                if pheeno:
                    
                    #Lane Tracking Function call
                    error = Lane_tracking(stream.array)
                    
                
                    if error != None:
                    
                        error_string = 'PHEENO'+';'+str(int(error))+':'
                        
                        print(error_string)
                        message5 = error_string
                        # Send information.
                        usb.flush()
                        usb.write(message5)          
                        usb.flush()
                        error_string = 'None'

                #print time.time()-start


                
                # stream.array contains the image array in bgr order!
                
                #cv2.imshow('Video gauss', gauss_gray)
                #cv2.imshow('Video', stream.array)



                # Break if cv2 window is open using "q" button.
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

                # Reset the array for next capture.
                stream.seek(0)
                stream.truncate()
        except KeyboardInterrupt:
            usb.flush()
            usb.close()
            sys.exit("Successfully Exited")
