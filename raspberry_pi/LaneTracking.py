import picamera
import picamera.array
import cv2
import numpy as np
import math
from image_processing import grayscale,canny,gaussian_blur,sobel,\
     region_of_interest, Counters_lane, draw_lines,\
     hough_lines_p,threshold, Counters_signal, Counters_Pheeno,\
     seg_intersect, movingAverage
from capture_image import image_capture
import time

error = 0
prev_error = 0

lane_bound_upper = np.array([180, 255, 255], dtype="uint8")
lane_bound_lower = np.array([0, 0, 170], dtype="uint8")

center_center_point = 0

def birdeye(img, display=False):
    """
    Apply perspective transform to input frame to get the bird's eye view.
    :param img: input color frame
    :param verbose: if True, show the transformation result
    :return: warped image, and both forward and backward transformation matrices
    """
    h, w = img.shape[:2]

    src = np.float32([[w, h],    # br
                      [0, h],    # bl
                      [2*w/10, 2*h/3],   # tl
                      [8*w/10, 2*h/3]])  # tr
    dst = np.float32([[w, h],       # br
                      [0, h],       # bl
                      [0, 0],       # tl
                      [w, 0]])      # tr

    M = cv2.getPerspectiveTransform(src, dst)
    Minv = cv2.getPerspectiveTransform(dst, src)

    warped = cv2.warpPerspective(img, M, (w, h), flags=cv2.INTER_LINEAR)

    if display:
        cv2.imshow('input',img)
        #cv2.imshow('warped',warped)

    return warped, M, Minv




def Lane_tracking(image , display = False):

    start = time.time()

    global prev_error

    a=0.20
    left_line = (0,0,0,0)
    right_line = (0,0,0,0)

    # Converting to HSV
    hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)

    # Masking red color
    mask_lane = cv2.inRange(hsv,lane_bound_lower,lane_bound_upper)
    mask_lane_image = cv2.bitwise_and(image,image,mask=mask_lane)

    # Converting to grayscale
    gray = grayscale(mask_lane_image)

    #Applying Gaussian Blur
    gaussian = gaussian_blur(gray , 5)

    #thresholding
    thresh = threshold(gaussian)


    # Sperating the region of interest

    h,w,_ = image.shape

    lower_left = [0,h]
    lower_right = [w,h]
    top_left = [0,2*h/3]
    top_right = [w,2*h/3]
    vertices = [np.array([lower_left,top_left,top_right,lower_right],dtype=np.int32)]
    vertices = [np.array([lower_left,top_left,top_right,lower_right],dtype=np.int32)]
    roi_image = region_of_interest(thresh, vertices)

    # Canny canny_edges

    low_threshold = 100
    high_threshold = 200
    canny_edges = canny(roi_image,low_threshold,high_threshold)


    #Seperating the left and right line of the lane using histogram

    histogram = np.sum(roi_image[h//2:h,:],axis = 0)
    non_zero_elements = np.nonzero(histogram)
    ch = np.sum(non_zero_elements)
    l = np.size(non_zero_elements)
    #print'nonzero elements ',ch//l
    midpoint = len(histogram) // 2
    #midpoint = ch // l
    leftx_base = np.argmax(histogram[:midpoint])
    rightx_base = np.argmax(histogram[midpoint:]) + midpoint

    #Seperating the right line in a seperate image
    lower_left = [midpoint,h]
    lower_right = [w,h]
    top_left = [midpoint,0]
    top_right = [w,0]
    vertices = [np.array([lower_left,top_left,top_right,lower_right],dtype=np.int32)]
    right = region_of_interest(canny_edges, vertices)

    #Seperating the left line in a seperate image
    lower_left = [0,h]
    lower_right = [midpoint,h]
    top_left = [0,0]
    top_right = [midpoint,0]
    vertices = [np.array([lower_left,top_left,top_right,lower_right],dtype=np.int32)]
    left = region_of_interest(canny_edges, vertices)

    # Define an imaginary horizontal line in the center of the screen
    # and at the bottom of the image, to extrapolate determined segment
    up_line_point1 = np.array( [0, int((2*h/3))] )
    up_line_point2 = np.array( [int(w), int((2*h/3))] )
    down_line_point1 = np.array( [0, int(h)] )
    down_line_point2 = np.array( [int(w), int(h)] )

    # Drawing the half line on the image
    cv2.line(image,(up_line_point1[0],up_line_point1[1]),(up_line_point2[0],up_line_point2[1]),(255,255,255),2)

    # Drawing the bottom line on the image
    #cv2.line(roi_image,(down_line_point1[0],down_line_point1[1]),(down_line_point2[0],down_line_point2[1]),(255,255,255),2)

    ### Fing the lane lines in the left lane image
    # Standard hough transform to get the rho and the theta of the line

    left_lines = cv2.HoughLines(left,2,np.pi/180,51)

    l_x1 = 0
    l_x2 = 0
    l_y1 = 0
    l_y2 = 0
    l_x1_a = 0
    l_x2_a = 0
    l_y1_a = 0
    l_y2_a = 0
    l_avgx1 = 0
    l_avgy1 = 0
    l_avgx2 = 0
    l_avgy2 = 0
    l_x1f = 0
    l_x2f = 0

    global l_x1,l_x2,l_y1,l_y2,l_x1f,l_x2f,l_avgx1,l_avgy1,l_avgx2,l_avgy2

    # proceed only if there ae lines found
    if left_lines != None:

       for left_line in left_lines:
           #Converting polar co-ordinates to cartesian
           for l_rho,l_theta in left_line:

               l_a = np.cos(l_theta)
               l_b = np.sin(l_theta)
               l_x0 = l_a*l_rho
               l_y0 = l_b*l_rho
               l_x1 = int(l_x0 + 1000*(-l_b))
               l_y1 = int(l_y0 + 1000*(l_a))
               l_x2 = int(l_x0 - 1000*(-l_b))
               l_y2 = int(l_y0 - 1000*(l_a))

           # Finding an average of all the lines found
           l_x1_a += l_x1
           l_y1_a += l_y1
           l_x2_a += l_x2
           l_y2_a += l_y2

       l_avgx1 = l_x1_a/len(left_lines)
       l_avgx2 = l_x2_a/len(left_lines)
       l_avgy1 = l_y1_a/len(left_lines)
       l_avgy2 = l_y2_a/len(left_lines)

       left_line = (l_avgx1, l_avgy1, l_avgx2, l_avgy2)

       # Find the intersection of dominant lane with an imaginary horizontal line
       # in the middle of the image and at the bottom of the image.
       if l_avgx1 != l_avgx2:

           slope_left = ((l_avgy1 - l_avgy2)*(1.0))/(l_avgx1 - l_avgx2)
           y = h
           l_x1f = (y - l_avgy1)/slope_left + l_avgx1
           y = 2*h/3
           l_x2f = (y - l_avgy1)/slope_left + l_avgx1

           cv2.line(image,(int(l_x1f),h),(int(l_x2f),2*h/3),(255,255,255),2)


        #   print 'Left Line', left_line,' Slope ',slope_left


##    lines = cv2.HoughLinesP(right,2,np.pi/180, 61, np.array([]), 21,21)
##    for lines
    ### Fing the lane lines in the left lane image
    # Standard hough transform to get the rho and the theta of the line
    right_lines = cv2.HoughLines(right,2,np.pi/180,51)

    r_x1 = 0
    r_x2 = 0
    r_y1 = 0
    r_y2 = 0
    r_x1_a = 0
    r_x2_a = 0
    r_y1_a = 0
    r_y2_a = 0
    l_avgx1 = 0
    l_avgy1 = 0
    l_avgx2 = 0
    l_avgy2 = 0
    r_x1f = 0
    r_x2f = 0

    global r_x1,r_x2,r_y1,r_y2,r_x1f,r_x2f,r_avgx1,r_avgy1,r_avgx2,r_avgy2

    # proceed only if there ae lines found
    if right_lines != None:

       for right_line in right_lines:
           #Converting polar co-ordinates to cartesian
           for r_rho,r_theta in right_line:

               r_a = np.cos(r_theta)
               r_b = np.sin(r_theta)
               r_x0 = r_a*r_rho
               r_y0 = r_b*r_rho
               r_x1 = int(r_x0 + 1000*(-r_b))
               r_y1 = int(r_y0 + 1000*(r_a))
               r_x2 = int(r_x0 - 1000*(-r_b))
               r_y2 = int(r_y0 - 1000*(r_a))

           # Finding an average of all the lines found
           r_x1_a += r_x1
           r_y1_a += r_y1
           r_x2_a += r_x2
           r_y2_a += r_y2

       r_avgx1 = r_x1_a/len(right_lines)
       r_avgx2 = r_x2_a/len(right_lines)
       r_avgy1 = r_y1_a/len(right_lines)
       r_avgy2 = r_y2_a/len(right_lines)

       right_line = (r_avgx1, r_avgy1, r_avgx2, r_avgy2)
       #cv2.line(image,(int(r_x1),r_avgy1),(int(r_x2),r_avgy2),(255,255,255),2)

       # Find the intersection of dominant lane with an imaginary horizontal line
       # in the middle of the image and at the bottom of the image.
       if r_avgx1 != r_avgx2:
           
           slope_right = ((r_avgy1 - r_avgy2)*(1.0))/(r_avgx1 - r_avgx2)
           y = h
           r_x1f = (y - r_avgy1)/slope_right + r_avgx1
           y = 2*h/3
           r_x2f = (y - r_avgy1)/slope_right + r_avgx1

           cv2.line(image,(int(r_x1f),h),(int(r_x2f),2*h/3),(255,255,255),2)


           #print 'Right line ', right_line , ' Slope ' , slope_right


    global error ,center_center_point
    #Computing the center of the lane
    if r_x2f and l_x2f:

        up_center_point = (int((r_x2f + l_x2f)/2) , 2*h/3)
        down_center_point = (int((r_x1f + l_x1f)/2) , h)
        center_center_point = ((up_center_point[0] + down_center_point[0])/2 , (up_center_point[1] + down_center_point[1])/2)
        #center_center_point = movingAverage(center_center_point,center_center_point_1[0])
        #print up_center_point,'    ',down_center_point

        #Computing the error

        img_center = ((h/2),(w/2))
        cv2.line(image,(img_center[1],h/2),(img_center[1],h),(255,255,255),2)
        slope_normal = 1111
        if up_center_point[0] != down_center_point[0]:
            slope_center = ((up_center_point[1] - down_center_point[1])*(1.0))/(up_center_point[0] - down_center_point[0])
            #print ' Slope center ' , slope_center
            x_intersection_point = w/2
            y_intersection_point = (x_intersection_point - down_center_point[0])*slope_center + down_center_point[1]

            length_base = y_intersection_point - h/2
            length_height = r_x2f - w/2

            angle = math.tan(length_height/length_base)
            #print 'angle',angle
            #angle = math.degrees(angle)

        error = center_center_point[0] - (w/2)
        cv2.line(image,(up_center_point),(down_center_point),(255,255,255),2)

    else:
        error = None

    if(error !=None):
        #print 'error b4', error
        a = 0.3
        error = error*(a)+ prev_error*(1-a)
        #print ' error  ',error,'  angle  ',angle
        prev_error = error

    if (display):
        cv2.imshow('roi', roi_image)
        cv2.imshow('image', image)
        cv2.imshow('imager', right)
        cv2.imshow('imagel', left)
    #print time.time()-start
    #print error
    return (error)




def lane_processing(img , display = False):

    
    #birds_eye,_,_ = birdeye(img,display)
    thresh = cv2.threshold(img ,150, 255, cv2.THRESH_BINARY)[1]    

    # Start the lane detection process
    lane_error = Lane_tracking(thresh, display)

    return lane_error


if __name__ == '__main__':

    mode = 'pi_camera'

    if mode == 'pi_camera':

        while True:
            # Get the input image
            frame = image_capture()

            # Start the lane detection process
            lane_error = lane_processing(frame,True)
    

          #  print lane_error

            #Display the detected lane on the frame
            #cv2.imshow('Input Image', blend)

            # Break if cv2 window is open by pressig "q" button.
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
