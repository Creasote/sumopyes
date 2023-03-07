#import the libraries
import cv2 as cv
import numpy as np

hsv_range_width = 10 # The width of the range (+/-) applied to HSV values for filtering
lower_S_value = 150
upper_S_value = 255
lower_V_value = 50
upper_V_value = 255

#helper fn, null callback
def nothing(x):
    pass


#Normalise
# Returns a tuple (lower_bound, upper_bound) for Hue
# within given range (0-180)
def normalise(val):
    #print("Normalising hue: ", val)
    return max(val-hsv_range_width,0),min(val+hsv_range_width, 180)

# Create_Mask(h)
# Takes Hue value, queries and builds lower and upper bound arrays
# then creates a mask using inRange on the HSV image.
# Note: All use same S (150-255) and V (50-255) range 
def create_mask(h):
    l,u = normalise(h)
    return cv.inRange(hsv, np.array([l,lower_S_value,lower_V_value]),np.array([u,upper_S_value,upper_V_value]))

#set up work canvas
cv.namedWindow('canvas')

#read the image
img = cv.imread("./images/colour.jpg")

#convert the BGR image to HSV colour space
hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

#create the menu / sliders
cv.createTrackbar('Target (1)','canvas',10,180,nothing) # Target 1 (lower range red)
cv.createTrackbar('Target (2)','canvas',170,180,nothing) # Target 2 (upper range red)
cv.createTrackbar('Self (front)','canvas',105,255,nothing) # Self 1 (blue)
cv.createTrackbar('Self (rear)','canvas',45,255,nothing) # Self 2 (green)


while(1):
    cv.imshow('canvas',img)
    k = cv.waitKey(1) & 0xFF
    if k == 27:
        break

    # get current positions of trackbars
    target1_mask = create_mask(cv.getTrackbarPos('Target (1)','canvas'))
    target2_mask = create_mask(cv.getTrackbarPos('Target (2)','canvas'))

    # Combine the lower and upper bound Target filters
    target_mask = cv.bitwise_or(target1_mask,target2_mask)

    self_f_mask = create_mask(cv.getTrackbarPos('Self (front)','canvas'))
    self_r_mask = create_mask(cv.getTrackbarPos('Self (rear)','canvas'))

    # Combine the front and rear Self filters
    self_mask = cv.bitwise_or(self_f_mask,self_r_mask)

    # For demo purposes, create a single mask that includes both target and self
    total_mask = cv.bitwise_or(self_mask,target_mask)
    res = cv.bitwise_and(img, img, mask=total_mask)

    #create resizable windows for displaying the images
    cv.namedWindow("res", cv.WINDOW_NORMAL)
    #cv.namedWindow("hsv", cv.WINDOW_NORMAL)
    cv.namedWindow("mask", cv.WINDOW_NORMAL)

    #display the images
    cv.imshow("mask", total_mask)
    #cv.imshow("hsv", hsv)
    cv.imshow("res", res)


cv.destroyAllWindows()




# #create a mask for green colour using inRange function
# mask = cv.inRange(hsv, lower_green, upper_green)

# #perform bitwise and on the original image arrays using the mask
# res = cv.bitwise_and(img, img, mask=mask)

# #create resizable windows for displaying the images
# cv.namedWindow("res", cv.WINDOW_NORMAL)
# cv.namedWindow("hsv", cv.WINDOW_NORMAL)
# cv.namedWindow("mask", cv.WINDOW_NORMAL)

# #display the images
# cv.imshow("mask", mask)
# cv.imshow("hsv", hsv)
# cv.imshow("res", res)

# if cv.waitKey(0):
#     cv.destroyAllWindows()