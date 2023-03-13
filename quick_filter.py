#import the libraries
import cv2 as cv
import numpy as np
import math
from simple_pid import PID
import com
import time

cap = cv.VideoCapture(0)

#TODO: Modify range of motors because we can only send a single 8 bit byte.
# Suggest to encode: motor speed as LS 6 bits (0-63), the next most significant bit (7) for motor indicator
# # (0 for left, 1 for right). Finally, the MSB (8) is a flag to indicate whether this is a motor speed update
#  (1) or something else (0), TBD. Easily filtered on the receiving end.
# ZUMO specific components
max_speed = 100  # We set a max speed of 100. This is scaled on receipt to the robot maximum of 400.
zero_point = 0 # What is the motor zero speed?
motor_speed = [zero_point, zero_point]


# Initialise PID controller. Variables can be tuned at run-time.
p_value = 1
i_value = 0.1
d_value = 0.05
pid = PID(p_value, i_value, d_value, setpoint=0)
pid.sample_time = 0.0334 # 30 FPS. Update if camera changes.
pid.output_limits = (-max_speed, max_speed) 
pid_img = np.zeros((50,50,1), np.uint8) 

# Disabled as manually setting mask values
# hsv_range_width = 10 # The width of the range (+/-) applied to HSV values for filtering
# lower_S_value = 150
# upper_S_value = 255
# lower_V_value = 50
# upper_V_value = 255

#helper fn, null callback
def nothing(x):
    pass


#Normalise
# Returns a tuple (lower_bound, upper_bound) for Hue
# within given range (0-180)
# def normalise(val):
#     return max(val-hsv_range_width,0),min(val+hsv_range_width, 180)

# Create_Mask(h)
# Takes Hue value, queries and builds lower and upper bound arrays
# then creates a mask using inRange on the HSV image.
# Note: All use same S (150-255) and V (50-255) range 
# def create_mask(h):
#     l,u = normalise(h)
#     return cv.inRange(hsv, np.array([l,lower_S_value,lower_V_value]),np.array([u,upper_S_value,upper_V_value]))

# Find_CoM(mask)
# Takes a mask (binary image) and find the centre of mass of the largest blob.
# Returns the coordinates as int: x,y
def find_com(mask):
    # Find Centre of Mass of Target
    # calculate moments of binary image

    # Creating erosion kernel
    kernel = np.ones((25, 25), np.uint8)
  
    # Using cv.erode() method 
    mask = cv.erode(mask, kernel) 

    target_moment = cv.moments(mask, True)

    if target_moment["m00"] != 0:     # Confirm a non-zero area
        # calculate x,y coordinate of center
        cX = int(target_moment["m10"] / target_moment["m00"])
        cY = int(target_moment["m01"] / target_moment["m00"])
    else: # Blob not identified, so return a Nonetype
            cX = cY = None
    return cX, cY

#set up work SumoPyes
cv.namedWindow('SumoPyes')
cv.namedWindow('PID Controller')
cv.namedWindow('Target')
cv.namedWindow('Self')

#create the menu / sliders
cv.createTrackbar('Target Hue (1) from','Target',0,180,nothing) # Target 1 (lower range red)
cv.createTrackbar('Target Hue (1) to','Target',20,180,nothing) # Target 1 (lower range red)
cv.createTrackbar('Target Sat (1) from','Target',150,255,nothing) # Target 1 (lower range red)
cv.createTrackbar('Target Sat (1) to','Target',255,255,nothing) # Target 1 (lower range red)
cv.createTrackbar('Target Val (1) from','Target',50,255,nothing) # Target 1 (lower range red)
cv.createTrackbar('Target Val (1) to','Target',255,255,nothing) # Target 1 (lower range red)
cv.createTrackbar('Target Hue (2) from','Target',160,180,nothing) # Target 1 (lower range red)
cv.createTrackbar('Target Hue (2) to','Target',180,180,nothing) # Target 1 (lower range red)
cv.createTrackbar('Target Sat (2) from','Target',150,255,nothing) # Target 1 (lower range red)
cv.createTrackbar('Target Sat (2) to','Target',255,255,nothing) # Target 1 (lower range red)
cv.createTrackbar('Target Val (2) from','Target',50,255,nothing) # Target 1 (lower range red)
cv.createTrackbar('Target Val (2) to','Target',255,255,nothing) # Target 1 (lower range red)

cv.createTrackbar('Self (rear)','Self',105,180,nothing) # Self 1 (blue)
cv.createTrackbar('Self (front)','Self',25,180,nothing) # Self 2 (yellow)

# create the PID controller adjustments
cv.createTrackbar('P','PID Controller',10,100,nothing) # Proportional, desired value 1, scale by factor of 10 (0.1 to 10)
cv.createTrackbar('I','PID Controller',10,100,nothing) # Integral, desired value 0.1, scale by factor of 100 (0.01 to 1)
cv.createTrackbar('D','PID Controller',5,100,nothing) # Differential, desired value 0.05 scale by factor of 100 (0.01 to 1)


# Set up Dict to hold overlay text
object_coords = {}

while(1):
    # Take each frame
    _, img = cap.read()

    #convert the BGR image to HSV colour space for object detection
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

    k = cv.waitKey(1) & 0xFF
    if k == 27:
        break

    # get current positions of trackbars
    # target1_mask = create_mask(cv.getTrackbarPos('Target (1)','SumoPyes'))
    # target2_mask = create_mask(cv.getTrackbarPos('Target (2)','SumoPyes'))

    # MANUALLY create masks to reduce UI complexity. Use recal tool if necessary to find new ranges.
    target1_mask = cv.inRange(hsv, np.array([0,150,50]),np.array([20,255,255]))
    target2_mask = cv.inRange(hsv, np.array([160,150,50]),np.array([180,255,255]))
    self_r_mask = cv.inRange(hsv, np.array([50,150,50]),np.array([150,255,255])) # blue
    # self_f_mask = cv.inRange(hsv, np.array([15,0,215]),np.array([40,255,255])) # yellow
    self_f_mask = cv.inRange(hsv, np.array([15,70,180]),np.array([40,255,255])) # yellow

    # Combine the lower and upper bound Target filters
    target_mask = cv.bitwise_or(target1_mask,target2_mask)

    # Find Centre of Mass of Target
    target_x, target_y = find_com(target_mask)
    if (target_x):
        object_coords["target"] = [target_x, target_y]
    else:
        object_coords.pop("target", None)


    # self_f_mask = create_mask(cv.getTrackbarPos('Self (front)','SumoPyes'))
    #TODO: See if some anti-jitter is possible?
    self_x, self_y = find_com(self_f_mask)
    if (self_x):
        object_coords["self_front"] = [self_x, self_y]
    else:
        object_coords.pop("self_front", None)

    # self_r_mask = create_mask(cv.getTrackbarPos('Self (rear)','SumoPyes'))
    self_x, self_y = find_com(self_r_mask)
    if (self_x):
        object_coords["self_rear"] = [self_x, self_y]
    else:
        object_coords.pop("self_rear", None)

    # Combine the front and rear Self filters
    self_mask = cv.bitwise_or(self_f_mask,self_r_mask)

    # For demo purposes, create a single mask that includes both target and self
    total_mask = cv.bitwise_or(self_mask,target_mask)
    res = cv.bitwise_and(img, img, mask=total_mask)

    # Finally add enhancements overlay before display
    x_t,y_t = object_coords.pop("target",[None,None])
    if x_t:
        # put text and highlight the center
        cv.circle(img, (x_t, y_t), 5, (255, 255, 255), -1)
        cv.putText(img, "TARGET", (x_t - 25, y_t - 25),cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    
    x_f,y_f = object_coords.pop("self_front",[None,None])
    x_r,y_r = object_coords.pop("self_rear",[None,None])
    if (x_f and x_r):
        # Create a vector representing SELF orientation
        vec_self = [(x_r,y_r),(x_f,y_f)]
        cv.circle(img, (x_f, y_f), 5, (255, 255, 255), -1)
        cv.putText(img, "SELF", (x_f - 25, y_f - 25),cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        # print the line
        cv.line(img,vec_self[0],vec_self[1], (255,255,255), 2)
    elif x_f:
        # just put a dot for the front
        cv.circle(img, (x_f, y_f), 5, (255, 255, 255), -1)
        cv.putText(img, "SELF", (x_f - 25, y_f- 25),cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
    elif x_r:
        # just put a dot for the rear
        cv.circle(img, (x_r, y_r), 5, (255, 255, 255), -1)
        cv.putText(img, "SELF", (x_r - 25, y_r - 25),cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    # Reset speeds to zero
    motor_speed[0] = zero_point
    motor_speed[1] = zero_point

    if (x_f and x_r and x_t):
        # Create a vector representing Path to Target
        vec_path = [(x_f,y_f),(x_t,y_t)]
        cv.line(img,vec_path[0],vec_path[1], (64,64,255), 2)
        cv.putText(img, "PATH", (int((vec_path[0][0]+vec_path[1][0])/2), int((vec_path[0][1]+vec_path[1][1])/2)),cv.FONT_HERSHEY_SIMPLEX, 0.5, (64, 64, 255), 2)

        # Find the angle between them.
        # If they are parallel, angle = 0
        # If they are at 3 o'clock, angle = +45
        # If they are at 9 o'clock, angle = -45
        # angle = math.degrees(math.atan2(vec_path[0][1]-vec_path[1][1],vec_path[0][0]-vec_path[1][0]))
        path_angle = math.degrees(math.atan2(vec_path[0][1]-vec_path[1][1],vec_path[0][0]-vec_path[1][0]))
        robot_angle = math.degrees(math.atan2(y_f-y_r, x_f-x_r))
        angle = (180 + robot_angle)%180 - (180+path_angle)%180
        # print("Angles: robot: ", robot_angle)
        # print(" path: ", path_angle)
        print(" vector: ", angle)
        cv.putText(img, str(angle), (10,10),cv.FONT_HERSHEY_SIMPLEX, 0.5, (64, 64, 255), 2)
        # time.sleep(0.5)

        # Using the angle, determine commands for the robot
        # 
        #Sets both motors to MAX_SPEED, and then if PID is -ve, subtract the value from 
        # the left motor, and if it is +ve subtract from the right. 
        #
        # Target found
        output = pid(angle)
        if output > 0: 
            motor_speed[0] = max_speed - output
            motor_speed[1] = max_speed   
        else: 
            motor_speed[0] = max_speed 
            motor_speed[1] = max_speed + output 

    # Now send this to the robot
    print("Motor settings: ", motor_speed)
    com.send_speed(motor_speed)
    print("From robot: ",com.read())
            

    # Update PID parameters
    p_value = cv.getTrackbarPos('P','PID Controller')/10
    i_value = cv.getTrackbarPos('I','PID Controller')/100
    d_value = cv.getTrackbarPos('D','PID Controller')/100
    pid.tunings = (p_value, i_value, d_value) # updated via sliders
    print("PID updated to: ", p_value, i_value, d_value)

    #create resizable windows for displaying the images
    cv.namedWindow("res", cv.WINDOW_NORMAL)
    cv.namedWindow("mask", cv.WINDOW_NORMAL)

    #
    target_display = cv.bitwise_and(img, img, mask=target_mask)
    self_display = cv.bitwise_and(img, img, mask=self_mask)

    #display the images
    cv.imshow("mask", total_mask)
    cv.imshow('SumoPyes',img)
    cv.imshow("res", res)
    cv.imshow("Target", target_display)
    cv.imshow("Self", self_display)
    cv.imshow("PID Controller", pid_img)

    #fps = cap.get(cv.CAP_PROP_FPS)
    #print("Frames per second using video.get(cv2.CAP_PROP_FPS) : {0}".format(fps))


cv.destroyAllWindows()