import numpy as np
import cv2
from image_processing import grayscale,canny,gaussian_blur,sobel,\
     region_of_interest, Counters_lane, draw_lines,\
     hough_lines_p,threshold, Counters_signal, Counters_Pheeno,seg_intersect

img = cv2.imread('curved2.jpg',0)

#thresholding
thresh = cv2.threshold(img ,235, 255, cv2.THRESH_BINARY)[1]
gauss = cv2.GaussianBlur(thresh, (5, 5), 0)

imshape = img.shape
lower_left = [imshape[1]/100,6*imshape[0]/7]
lower_right = [imshape[1]-imshape[1]/100,6*imshape[0]/7]
top_left = [imshape[1]/2-imshape[1]/1,2*imshape[0]/7]
top_right = [imshape[1]/2+imshape[1]/1,2*imshape[0]/7]
vertices = [np.array([lower_left,top_left,top_right,lower_right],dtype=np.int32)]
roi_image = region_of_interest(gauss, vertices)


h , w = roi_image.shape

left_line_img = np.zeros((h, w), dtype=np.uint8)
right_line_img = np.zeros((h, w), dtype=np.uint8)

histogram = np.sum(roi_image[h//2:h,:],axis = 0)

midpoint = len(histogram) // 2

leftx_base = np.argmax(histogram[:midpoint])
rightx_base = np.argmax(histogram[midpoint:]) + midpoint


# Canny canny_edges

low_threshold = 100
high_threshold = 200
canny_edges = canny(roi_image,low_threshold,high_threshold)



lower_left = [midpoint,h]
lower_right = [w,h]
top_left = [midpoint,0]
top_right = [w,0]
vertices = [np.array([lower_left,top_left,top_right,lower_right],dtype=np.int32)]
right = region_of_interest(canny_edges, vertices)



lower_left = [0,h]
lower_right = [midpoint,h]
top_left = [0,0]
top_right = [midpoint,0]
vertices = [np.array([lower_left,top_left,top_right,lower_right],dtype=np.int32)]
left = region_of_interest(canny_edges, vertices)

#
# # Define an imaginary horizontal line in the center of the screen
# # and at the bottom of the image, to extrapolate determined segment
# imgHeight, imgWidth = (img.shape[0], img.shape[1])
# upLinePoint1 = np.array( [0, int(imgHeight - (imgHeight/2))] )
# upLinePoint2 = np.array( [int(imgWidth), int(imgHeight - (imgHeight/2))] )
# downLinePoint1 = np.array( [0, int(imgHeight)] )
# downLinePoint2 = np.array( [int(imgWidth), int(imgHeight)] )

lines_left = cv2.HoughLines(left,2,np.pi/180,70)
x1_l =0
x2_l =0
y1_l=0
y2_l=0
global x1_l,x2_l,y1_l,y2_l
for line in lines_left:

    for rho,theta in line:
        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a*rho
        y0 = b*rho
        x1_l += int(x0 + 1000*(-b))
        y1_l += int(y0 + 1000*(a))
        x2_l += int(x0 - 1000*(-b))
        y2_l += int(y0 - 1000*(a))
        #cv2.line(roi_image,(x1_l,y1_l),(x2_l,y2_l),(255,255,255),2)

avgx1_l = x1_l/len(lines_left)
avgx2_l = x2_l/len(lines_left)
avgy1_l = y1_l/len(lines_left)
avgy2_l = y2_l/len(lines_left)



cv2.line(roi_image,(avgx1_l,avgy1_l),(avgx2_l,avgy2_l),(255,255,255),2)


slope_left = (avgy1_l - avgy2_l)/(avgx1_l - avgx2_l)

f = (0 - avgy2_l)/slope_left + avgx2_l
g = (h - avgy1_l)/slope_left + avgx1_l

y = 0
new_x1_l = (y - avgy2_l)/slope_left + avgx2_l
y = h
new_x2_l = (y - avgy1_l)/slope_left + avgx1_l

new_slope = (h - avgy2_l)/(new_x2_l - avgx1_l)

print f ,'  ', g, new_x1_l , '  ', new_x2_l

cv2.line(roi_image,(new_x1_l,1),(new_x2_l,h),(255,255,255),2)


lines_right = cv2.HoughLines(right,2,np.pi/180,100)
x1 =0
x2 =0
y1=0
y2=0
global x1,x2,y1,y2
for line in lines_right:

    for rho,theta in line:
        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a*rho
        y0 = b*rho
        x1 += int(x0 + 1000*(-b))
        y1 += int(y0 + 1000*(a))
        x2 += int(x0 - 1000*(-b))
        y2 += int(y0 - 1000*(a))
        #cv2.line(roi_image,(x1,y1),(x2,y2),(255,255,255),2)

avgx1 = x1/len(lines_right)
avgx2 = x2/len(lines_right)
avgy1 = y1/len(lines_right)
avgy2 = y2/len(lines_right)

cv2.line(roi_image,(avgx1,avgy1),(avgx2,avgy2),(255,255,255),2)

slope_right = (avgy2 - avgy1)/(avgx2 - avgx1)

y = 0
x1 = (y - avgy1)/slope_right + avgx1
y = h
x2 = (y - avgy1)/slope_right + avgx1

#cv2.line(roi_image,(x1,0),(x2,h),(255,255,255),2)




cv2.imshow('image',img)
#cv2.imshow('thresh',gauss)
cv2.imshow('roi',roi_image)
#cv2.imshow('canny', canny_edges)
#cv2.imshow('lines', line_img)
cv2.imshow('left',left)
#cv2.imshow('right', right)
cv2.waitKey(0)
cv2.destroyAllWindows()
