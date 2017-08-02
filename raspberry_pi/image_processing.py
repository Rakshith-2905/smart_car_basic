import cv2
import numpy as np
import math

prev_w_w =0
prev_h_w=0

prev_w_y=0
prev_h_y=0

avgLeft = (0, 0, 0, 0)
avg_middle = (0, 0, 0, 0)
avgRight = (0, 0, 0, 0)


def grayscale(img):
    """Applies the Grayscale transform
    This will return an image with only one color channel
    but NOTE: to see the returned image as grayscale
    (assuming your grayscaled image is called 'gray')
    you should call plt.imshow(gray, cmap='gray')"""
    return cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    # Or use BGR2GRAY if you read an image with cv2.imread()
    # return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)


def canny(img, low_threshold, high_threshold):
    """Applies the Canny transform"""
    output = cv2.Canny(img, low_threshold, high_threshold)
    return output



def gaussian_blur(img, kernel_size):
    """Applies a Gaussian Noise kernel"""
    return cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)

def sobel(img,dst,xorder,yorder):
    """ Applies sobel filter"""
    return cv2.sobel(img,dst,Xorder,yorder)


def threshold(img):
    "Applies thresholding"
    thres1 = cv2.threshold(img ,130, 255, cv2.THRESH_BINARY)[1]
    #kernal = np.ones((17,17),np.uint8)
    #thres = cv2.erode(thres1,None , iterations=2)
    return thres1 #cv2.dilate(thres, None, iterations=2)






def region_of_interest(img, vertices):
    """
    Applies an image mask.
    Only keeps the region of the image defined by the polygon
    formed from `vertices`. The rest of the image is set to black.
    """
    #defining a blank mask to start with
    mask = np.zeros_like(img)

    #defining a 3 channel or 1 channel color to fill the mask with depending on the input image
    if len(img.shape) > 2:
        channel_count = img.shape[2]  # i.e. 3 or 4 depending on your image
        ignore_mask_color = (255,) * channel_count
    else:
        ignore_mask_color = 255

    #filling pixels inside the polygon defined by "vertices" with the fill color
    cv2.fillPoly(mask, vertices, ignore_mask_color)

    #returning the image only where mask pixels are nonzero
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image



def perp( a ) :
    b = np.empty_like(a)
    b[0] = -a[1]
    b[1] = a[0]
    return b

# line segment a given by endpoints a1, a2
# line segment b given by endpoints b1, b2
# return 




def seg_intersect(a1,a2, b1,b2):
    da = a2-a1
    db = b2-b1
    dp = a1-b1
    dap = perp(da)
    denom = np.dot( dap, db)
    num = np.dot( dap, dp )
    return (num / denom.astype(float))*db + b1

def movingAverage(avg, new_sample, N=2):
    if (avg == 0):
        return new_sample
    avg -= avg / N;
    avg += new_sample / N;
    return avg;






def draw_lines(img, lines, color=[255, 0, 0], thickness=2):
    
    """
    NOTE: this is the function you might want to use as a starting point once you want to 
    average/extrapolate the line segments you detect to map out the full
    extent of the lane (going from the result shown in raw-lines-example.mp4
    to that shown in P1_example.mp4).  
    
    Think about things like separating line segments by their 
    slope ((y2-y1)/(x2-x1)) to decide which segments are part of the left
    line vs. the right line.  Then, you can average the position of each of 
    the lines and extrapolate to the top and bottom of the lane.
    
    This function draws `lines` with `color` and `thickness`.    
    Lines are drawn on the image inplace (mutates the image).
    If you want to make the lines semi-transparent, think about combining
    this function with the weighted_img() function below
    """
    # state variables to keep track of most dominant segment
    largestLeftLineSize = 0
    largestRightLineSize = 0
    largestLeftLine = (0,0,0,0)
    largestRightLine = (0,0,0,0)

    imshape = img.shape
    
    if lines is None:
        avgx1, avgy1, avgx2, avgy2 = avgLeft
        cv2.line(img, (int(avgx1), int(avgy1)), (int(avgx2), int(avgy2)), [255,255,255], 12) #draw left line
        avgx1, avgy1, avgx2, avgy2 = avgRight
        cv2.line(img, (int(avgx1), int(avgy1)), (int(avgx2), int(avgy2)), [255,255,255], 12) #draw right line
        return

    for line in lines:
        
   
        #print count
        for x1,y1,x2,y2 in line:
            
            size = math.hypot(x2 - x1, y2 - y1)
            slope = ((y2-y1)/(x2-x1))
            #print slope
            #cv2.line(img, (x1, y1), (x2, y2), color, thickness)
            # Filter slope based on incline and
            # find the most dominent segment based on length
            if (slope > 0.5): #right
                if (size > largestRightLineSize):
                    largestRightLine = (x1, y1, x2, y2)                    
                cv2.line(img, (x1, y1), (x2, y2), color, thickness)
                
            elif (slope < -1): #left
                if (size > largestLeftLineSize):
                    largestLeftLine = (x1, y1, x2, y2)
                cv2.line(img, (x1, y1), (x2, y2), color, thickness)

    # Define an imaginary horizontal line in the center of the screen
    # and at the bottom of the image, to extrapolate determined segment
    imgHeight, imgWidth = (img.shape[0], img.shape[1])
    upLinePoint1 = np.array( [0, int(imgHeight - (imgHeight/2))] )
    upLinePoint2 = np.array( [int(imgWidth), int(imgHeight - (imgHeight/2))] )
    downLinePoint1 = np.array( [0, int(imgHeight)] )
    downLinePoint2 = np.array( [int(imgWidth), int(imgHeight)] )
    #cv2.line(img, ((int(upLinePoint1[0])), (int(upLinePoint1[1]))), ((int(upLinePoint2[0])),(int(upLinePoint2[1]))), color, thickness)
    #cv2.line(img, ((int(downLinePoint1[0])), (int(downLinePoint1[1]))), ((int(downLinePoint2[0])),(int(downLinePoint2[1]))), color, thickness)

    # Find the intersection of dominant lane with an imaginary horizontal line
    # in the middle of the image and at the bottom of the image.
    p3 = np.array( [largestLeftLine[0], largestLeftLine[1]] )
    p4 = np.array( [largestLeftLine[2], largestLeftLine[3]] )
    upLeftPoint = seg_intersect(upLinePoint1,upLinePoint2, p3,p4)
    downLeftPoint = seg_intersect(downLinePoint1,downLinePoint2, p3,p4)

    #cv2.line(img, ((int(upLeftPoint[0])), (int(upLeftPoint[1]))), ((int(downLeftPoint[0])),(int(downLeftPoint[1]))), color, thickness)
    #cv2.line(img, ((int(downLinePoint[0])), (int(downLinePoint[1]))), ((int(downLinePoint[0])),(int(downLinePoint[1]))), color, thickness)

    if (math.isnan(upLeftPoint[0]) or math.isnan(downLeftPoint[0])):
        avgx1, avgy1, avgx2, avgy2 = avgLeft
        cv2.line(img, (int(avgx1), int(avgy1)), (int(avgx2), int(avgy2)), [255,255,255], 12) #draw left line
        avgx1, avgy1, avgx2, avgy2 = avgRight
        cv2.line(img, (int(avgx1), int(avgy1)), (int(avgx2), int(avgy2)), [255,255,255], 12) #draw right line
        return
    #cv2.line(img, (int(upLeftPoint[0]), int(upLeftPoint[1])), (int(downLeftPoint[0]), int(downLeftPoint[1])), [0, 0, 255], 8) #draw left line

        # Calculate the average position of detected left lane over multiple video frames and draw
    global avgLeft
    avgx1, avgy1, avgx2, avgy2 = avgLeft
    avgLeft = (movingAverage(avgx1, upLeftPoint[0]), movingAverage(avgy1, upLeftPoint[1]), movingAverage(avgx2, downLeftPoint[0]), movingAverage(avgy2, downLeftPoint[1]))
    avgx1, avgy1, avgx2, avgy2 = avgLeft
    cv2.line(img, (int(avgx1), int(avgy1)), (int(avgx2), int(avgy2)), [255,255,255], 12) #draw left line

    # Find the intersection of dominant lane with an imaginary horizontal line
    # in the middle of the image and at the bottom of the image.
    p5 = np.array( [largestRightLine[0], largestRightLine[1]] )
    p6 = np.array( [largestRightLine[2], largestRightLine[3]] )
    upRightPoint = seg_intersect(upLinePoint1,upLinePoint2, p5,p6)
    downRightPoint = seg_intersect(downLinePoint1,downLinePoint2, p5,p6)


    if (math.isnan(upRightPoint[0]) or math.isnan(downRightPoint[0])):
        avgx1, avgy1, avgx2, avgy2 = avgLeft
        cv2.line(img, (int(avgx1), int(avgy1)), (int(avgx2), int(avgy2)), [255,255,255], 12) #draw left line
        avgx1, avgy1, avgx2, avgy2 = avgRight
        cv2.line(img, (int(avgx1), int(avgy1)), (int(avgx2), int(avgy2)), [255,255,255], 12) #draw right line
        return
       #cv2.line(img, (int(upRightPoint[0]), int(upRightPoint[1])), (int(downRightPoint[0]), int(downRightPoint[1])), [0, 0, 255], 8) #draw left line

        # Calculate the average position of detected right lane over multiple video frames and draw
    global avgRight
    avgx1, avgy1, avgx2, avgy2 = avgRight
    avgRight = (movingAverage(avgx1, upRightPoint[0]), movingAverage(avgy1, upRightPoint[1]), movingAverage(avgx2, downRightPoint[0]), movingAverage(avgy2, downRightPoint[1]))
    avgx1, avgy1, avgx2, avgy2 = avgRight
    cv2.line(img, (int(avgx1), int(avgy1)), (int(avgx2), int(avgy2)), [255,255,255], 12) #draw left line
    ##

    # Calculate the lane center
    global avg_middle
    avgx1, avgy1, avgx2, avgy2 = avg_middle
    lane_up_middle = (upRightPoint + upLeftPoint)/2
    lane_down_middle = (downRightPoint + downLeftPoint)/2
    avg_middle = (movingAverage(avgx1, lane_up_middle[0]), movingAverage(avgy1, lane_up_middle[1]), movingAverage(avgx2, lane_down_middle[0]), movingAverage(avgy2, lane_down_middle[1]))
    #cv2.line(img, (int(lane_up_middle[0]), int(lane_up_middle[1])), (int(lane_down_middle[0]), int(lane_down_middle[1])), [0, 0, 255], 8)
    cv2.line(img, (int(avgx1), int(avgy1)), (int(avgx2), int(avgy2)), [0,255,255], 12) #draw left line
    img_center = (((img.shape[1]/2)-15),(img.shape[0]/2))
    lane_center = ((int(avg_middle[0]+avg_middle[2])/2),(int(avg_middle[1]+avg_middle[3])/2))
    return lane_center[0]-img_center[0]


def hough_lines_p(img,img2, rho, theta, threshold, min_line_len, max_line_gap):
    """
    `img` should be the output of a Canny transform.
    Returns an image with hough lines drawn.
    """
    lines = cv2.HoughLinesP(img, rho, theta, threshold, np.array([]), minLineLength=min_line_len, maxLineGap=max_line_gap)
    line_img = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)
    middle = draw_lines(img2, lines)

    return line_img ,middle


def Counters_lane(img,color):
    global prev_w_w
    global prev_h_w
    global prev_w_y
    global prev_h_y
    #print color
    #ret, thresholded_image = cv2.threshold(img, 0, 127, cv2.THRESH_BINARY)
    contours, hierarchy = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
    if len(contours) > 0:
        cnt = contours[0]
    for contour in contours:
         rect = cv2.minAreaRect(contour)
         [x,y,w,h] = cv2.boundingRect(contour)
         
         if cv2.contourArea(contour) >70:
            #cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),1)

            if color =='yellow':
                slope = (y-(y+h))/(x-(x+w))
            if color=='white':
                slope = (y-(y+h))/((x+w)-x)
            

           
            if slope < 0:# and abs(h-prev_h_w)<20 and abs(w-prev_w_w)<20:
               # print 'left' , abs(y-prev_y)
                cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),1)
                cv2.line(img, (x+w,y),(x,y+h), [255, 255, 0], 2)
                print 'h ',h,'prev_h ',prev_h_w,'abs_h', abs(h-prev_h_w),'w ',w,'prev_w ',prev_w_w,'abs_w', abs(w-prev_w_w)

                prev_w_w=w
                prev_h_w=h
            
            if slope > 0:# and abs(x-prev_w_y)<10:
                
                cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),1)
                cv2.line(img, (x,y),(x+w,y+h), [255, 255, 0], 2)
                #print 'w_left ',w,'prev_w ',prev_w_y,abs(w-prev_w_y)

                prev_w_y=w
                prev_h_y=h

def Counters_Pheeno(img):
    
    contours = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        
    if len(contours) > 0:
        for contour in contours:
    
            # Find the largest contour in the map
            cnt = max(contours, key=cv2.contourArea)
##            ((x,y),radius) = cv2.minEnclosingCircle(contour)
##            #M = cv2.moments(cnt)
##            #center =
##            if radius >5:
##                return x,y,radius
            rect = cv2.minAreaRect(cnt)
            [x,y,w,h] = cv2.boundingRect(contour)
         
            if cv2.contourArea(contour) >5:
                if w>15 and h>10:
                    return x,y,w,h


def Counters_signal(img,img2):
    
    contours = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        
    if len(contours) > 0:
        for contour in contours:

            rect = cv2.minAreaRect(contour)
            [x,y,w,h] = cv2.boundingRect(contour)
         
            if cv2.contourArea(contour) >5:
                if w>40 and h>20:
                    return x,y,w,h




                


