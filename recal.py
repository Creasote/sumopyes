#import the libraries
import cv2 as cv
import numpy as np

cap = cv.VideoCapture(0)


hsv_range_width = 10 # The width of the range (+/-) applied to HSV values for filtering

#helper fn, null callback
def nothing(x):
    pass


#set up work canvas
cv.namedWindow('canvas')

#create the menu / sliders
cv.createTrackbar('Hue (from)','canvas',10,180,nothing)
cv.createTrackbar('Hue (to)','canvas',30,180,nothing) 
cv.createTrackbar('Saturation (from)','canvas',150,255,nothing)
cv.createTrackbar('Saturation (to)','canvas',255,255,nothing)
cv.createTrackbar('Value (from)','canvas',50,255,nothing)
cv.createTrackbar('Value (to)','canvas',255,255,nothing) 



while(1):
    # Take each frame
    _, img = cap.read()

    #convert the BGR image to HSV colour space for object detection
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

    k = cv.waitKey(10) & 0xFF
    if k == 27:
        break

    # get current positions of trackbars
    hue_lower = cv.getTrackbarPos('Hue (from)','canvas')
    hue_upper = cv.getTrackbarPos('Hue (to)','canvas')
    saturation_lower = cv.getTrackbarPos('Saturation (from)','canvas')
    saturation_upper = cv.getTrackbarPos('Saturation (to)','canvas')
    value_lower = cv.getTrackbarPos('Value (from)','canvas')
    value_upper = cv.getTrackbarPos('Value (to)','canvas')

    mask = cv.inRange(hsv, np.array([hue_lower, saturation_lower, value_lower]),np.array([hue_upper, saturation_upper, value_upper]))


    # For demo purposes, create a single mask that includes both target and self
    # total_mask = cv.bitwise_or(self_mask,target_mask)
    res = cv.bitwise_and(img, img, mask=mask)


    #create resizable windows for displaying the images
    cv.namedWindow("res", cv.WINDOW_NORMAL)
    cv.namedWindow("mask", cv.WINDOW_NORMAL)

    #display the images
    cv.imshow("mask", mask)
    cv.imshow('canvas',img)
    cv.imshow("res", res)


cv.destroyAllWindows()