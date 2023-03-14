#import the libraries
import cv2 as cv
import numpy as np
import math
from simple_pid import PID
import com
import time
import pickle

cap = cv.VideoCapture(0)

# Load config
file = open('config.conf', 'rb')
config = pickle.load(file)
file.close()

# Set filter values
t1_h_f = config["t1_h_f"]  
t1_h_t = config["t1_h_t"]  
t1_s_f = config["t1_s_f"]  
t1_s_t = config["t1_s_t"]  
t1_v_f = config["t1_v_f"]  
t1_v_t = config["t1_v_t"]  
t2_h_f = config["t2_h_f"]  
t2_h_t = config["t2_h_t"]  
t2_s_f = config["t2_s_f"]  
t2_s_t = config["t2_s_t"]  
t2_v_f = config["t2_v_f"]  
t2_v_t = config["t2_v_t"]  
sf_h_f = config["sf_h_f"]  
sf_h_t = config["sf_h_t"]  
sf_s_f = config["sf_s_f"]  
sf_s_t = config["sf_s_t"]  
sf_v_f = config["sf_v_f"]  
sf_v_t = config["sf_v_t"]  
sr_h_f = config["sr_h_f"]  
sr_h_t = config["sr_h_t"]  
sr_s_f = config["sr_s_f"]  
sr_s_t = config["sr_s_t"]  
sr_v_f = config["sr_v_f"]  
sr_v_t = config["sr_v_t"]  


#TODO: Modify range of motors because we can only send a single 8 bit byte.
# Suggest to encode: motor speed as LS 6 bits (0-63), the next most significant bit (7) for motor indicator
# # (0 for left, 1 for right). Finally, the MSB (8) is a flag to indicate whether this is a motor speed update
#  (1) or something else (0), TBD. Easily filtered on the receiving end.
# ZUMO specific components
max_speed = 100  # We set a max speed of 100. This is scaled on receipt to the robot maximum of 400.
zero_point = 0 # What is the motor zero speed?
motor_speed = [zero_point, zero_point]


# Initialise PID controller. Variables can be tuned at run-time.
p_value = config["p_value"] # 1
i_value = config["i_value"] # 0.1
d_value = config["d_value"] #0.05
pid = PID(p_value, i_value, d_value, setpoint=0)
pid.sample_time = 0.0334 # 30 FPS. Update if camera changes.
pid.output_limits = (-max_speed, max_speed) 
MAX_ANGLE = 135 # Constraint on the maximum magnitude of angle fed to the PID (degrees)
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

# Constrain applies upper and lower bounds to a value
def constrain(val, min_val, max_val):
    return min(max_val, max(min_val, val))

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
cv.createTrackbar('Target Hue (1) from','Target',t1_h_f,180,nothing) # Target 1 (lower range red)
cv.createTrackbar('Target Hue (1) to','Target',t1_h_t,180,nothing) # Target 1 (lower range red)
cv.createTrackbar('Target Sat (1) from','Target',t1_s_f,255,nothing) # Target 1 (lower range red)
cv.createTrackbar('Target Sat (1) to','Target',t1_s_t,255,nothing) # Target 1 (lower range red)
cv.createTrackbar('Target Val (1) from','Target',t1_v_f,255,nothing) # Target 1 (lower range red)
cv.createTrackbar('Target Val (1) to','Target',t1_v_t,255,nothing) # Target 1 (lower range red)
cv.createTrackbar('Target Hue (2) from','Target',t2_h_f,180,nothing) # Target 2 (upper range red)
cv.createTrackbar('Target Hue (2) to','Target',t2_h_t,180,nothing) # Target 2 (upper range red)
cv.createTrackbar('Target Sat (2) from','Target',t2_s_f,255,nothing) # Target 2 (upper range red)
cv.createTrackbar('Target Sat (2) to','Target',t2_s_t,255,nothing) # Target 2 (upper range red)
cv.createTrackbar('Target Val (2) from','Target',t2_v_f,255,nothing) # Target 2 (upper range red)
cv.createTrackbar('Target Val (2) to','Target',t2_v_t,255,nothing) # Target 2 (upper range red)

cv.createTrackbar('Self Hue (front) from','Self',sf_h_f,180,nothing) # Self front (yellow)
cv.createTrackbar('Self Hue (front) to','Self',sf_h_t,180,nothing) # Self front (yellow)
cv.createTrackbar('Self Sat (front) from','Self',sf_s_f,255,nothing) # Self front (yellow)
cv.createTrackbar('Self Sat (front) to','Self',sf_s_t,255,nothing) # Self front (yellow)
cv.createTrackbar('Self Val (front) from','Self',sf_v_f,255,nothing) # Self front (yellow)
cv.createTrackbar('Self Val (front) to','Self',sf_v_t,255,nothing) # Self front (yellow)
cv.createTrackbar('Self Hue (rear) from','Self',sr_h_f,180,nothing) # Self front (blue)
cv.createTrackbar('Self Hue (rear) to','Self',sr_h_t,180,nothing) # Self front (blue)
cv.createTrackbar('Self Sat (rear) from','Self',sr_s_f,255,nothing) # Self front (blue)
cv.createTrackbar('Self Sat (rear) to','Self',sr_s_t,255,nothing) # Self front (blue)
cv.createTrackbar('Self Val (rear) from','Self',sr_v_f,255,nothing) # Self front (blue)
cv.createTrackbar('Self Val (rear) to','Self',sr_v_t,255,nothing) # Self front (blue)

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
    t1_h_f = cv.getTrackbarPos('Target Hue (1) from','Target')
    t1_h_t = cv.getTrackbarPos('Target Hue (1) to','Target')
    t1_s_f = cv.getTrackbarPos('Target Sat (1) from','Target')
    t1_s_t = cv.getTrackbarPos('Target Sat (1) to','Target')
    t1_v_f = cv.getTrackbarPos('Target Val (1) from','Target')
    t1_v_t = cv.getTrackbarPos('Target Val (1) to','Target')
    target1_mask = cv.inRange(hsv, np.array([t1_h_f,t1_s_f,t1_v_f]),np.array([t1_h_t,t1_s_t,t1_v_t]))

    t2_h_f = cv.getTrackbarPos('Target Hue (2) from','Target')
    t2_h_t = cv.getTrackbarPos('Target Hue (2) to','Target')
    t2_s_f = cv.getTrackbarPos('Target Sat (2) from','Target')
    t2_s_t = cv.getTrackbarPos('Target Sat (2) to','Target')
    t2_v_f = cv.getTrackbarPos('Target Val (2) from','Target')
    t2_v_t = cv.getTrackbarPos('Target Val (2) to','Target')
    target2_mask = cv.inRange(hsv, np.array([t2_h_f,t2_s_f,t2_v_f]),np.array([t2_h_t,t2_s_t,t2_v_t]))

    sf_h_f = cv.getTrackbarPos('Self Hue (front) from','Self')
    sf_h_t = cv.getTrackbarPos('Self Hue (front) to','Self')
    sf_s_f = cv.getTrackbarPos('Self Sat (front) from','Self')
    sf_s_t = cv.getTrackbarPos('Self Sat (front) to','Self')
    sf_v_f = cv.getTrackbarPos('Self Val (front) from','Self')
    sf_v_t = cv.getTrackbarPos('Self Val (front) to','Self')
    self_f_mask = cv.inRange(hsv, np.array([sf_h_f,sf_s_f,sf_v_f]),np.array([sf_h_t,sf_s_t,sf_v_t])) # yellow

    sr_h_f = cv.getTrackbarPos('Self Hue (rear) from','Self')
    sr_h_t = cv.getTrackbarPos('Self Hue (rear) to','Self')
    sr_s_f = cv.getTrackbarPos('Self Sat (rear) from','Self')
    sr_s_t = cv.getTrackbarPos('Self Sat (rear) to','Self')
    sr_v_f = cv.getTrackbarPos('Self Val (rear) from','Self')
    sr_v_t = cv.getTrackbarPos('Self Val (rear) to','Self')
    self_r_mask = cv.inRange(hsv, np.array([sr_h_f,sr_s_f,sr_v_f]),np.array([sr_h_t,sr_s_t,sr_v_t])) # blue

    # MANUALLY create masks to reduce UI complexity. Use recal tool if necessary to find new ranges.
    # target1_mask = cv.inRange(hsv, np.array([0,150,50]),np.array([20,255,255]))
    # target2_mask = cv.inRange(hsv, np.array([160,150,50]),np.array([180,255,255]))
    # self_r_mask = cv.inRange(hsv, np.array([50,150,50]),np.array([150,255,255])) # blue
    # # self_f_mask = cv.inRange(hsv, np.array([15,0,215]),np.array([40,255,255])) # yellow
    # self_f_mask = cv.inRange(hsv, np.array([15,70,180]),np.array([40,255,255])) # yellow

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
    # total_mask = cv.bitwise_or(self_mask,target_mask)
    # res = cv.bitwise_and(img, img, mask=total_mask)

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
        # If they are at 3 o'clock, angle = -45
        # If they are at 9 o'clock, angle = +45
        # angle = math.degrees(math.atan2(vec_path[0][1]-vec_path[1][1],vec_path[0][0]-vec_path[1][0]))
        # path_angle = math.degrees(math.atan2(vec_path[0][1]-vec_path[1][1],vec_path[0][0]-vec_path[1][0]))
        path_angle = math.degrees(math.atan2(vec_path[1][1]-vec_path[0][1],vec_path[1][0]-vec_path[0][0]))
        robot_angle = math.degrees(math.atan2(y_f-y_r, x_f-x_r))
        # robot_angle = math.degrees(math.atan2(y_r-y_f, x_r-x_f))
        # print("Angles: robot: ", robot_angle)
        # print(" path: ", path_angle)
        angle = path_angle - robot_angle
        angle = (angle + 180) % 360 - 180
        print(" vector: ", angle)
        cv.putText(img, str(angle), (10,10),cv.FONT_HERSHEY_SIMPLEX, 0.5, (64, 64, 255), 2)
        cv.putText(img, str(robot_angle), (10,30),cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 64, 64), 2)
        cv.putText(img, str(path_angle), (10,50),cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        # time.sleep(0.5)

        # Using the angle, determine commands for the robot
        # 
        #Sets both motors to MAX_SPEED, and then if PID is -ve, subtract the value from 
        # the left motor, and if it is +ve subtract from the right. 
        #
        # Target found
        # output = constrain(pid(angle),-MAX_ANGLE,MAX_ANGLE)
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
    # cv.namedWindow("res", cv.WINDOW_NORMAL)
    # cv.namedWindow("mask", cv.WINDOW_NORMAL)

    #
    target_display = cv.bitwise_and(img, img, mask=target_mask)
    self_display = cv.bitwise_and(img, img, mask=self_mask)

    #display the images
    # cv.imshow("mask", total_mask)
    cv.imshow('SumoPyes',img)
    # cv.imshow("res", res)
    cv.imshow("Target", target_display)
    cv.imshow("Self", self_display)
    cv.imshow("PID Controller", pid_img)

    #fps = cap.get(cv.CAP_PROP_FPS)
    #print("Frames per second using video.get(cv2.CAP_PROP_FPS) : {0}".format(fps))


# Before exiting, save current config to file.
config = {
    "t1_h_f" :t1_h_f, 
    "t1_h_t" :t1_h_t, 
    "t1_s_f" :t1_s_f, 
    "t1_s_t" :t1_s_t, 
    "t1_v_f" :t1_v_f, 
    "t1_v_t" :t1_v_t, 
    "t2_h_f" :t2_h_f, 
    "t2_h_t" :t2_h_t, 
    "t2_s_f" :t2_s_f, 
    "t2_s_t" :t2_s_t, 
    "t2_v_f" :t2_v_f, 
    "t2_v_t" :t2_v_t, 
    "sf_h_f" :sf_h_f, 
    "sf_h_t" :sf_h_t, 
    "sf_s_f" :sf_s_f, 
    "sf_s_t" :sf_s_t, 
    "sf_v_f" :sf_v_f, 
    "sf_v_t" :sf_v_t, 
    "sr_h_f" :sr_h_f, 
    "sr_h_t" :sr_h_t, 
    "sr_s_f" :sr_s_f, 
    "sr_s_t" :sr_s_t, 
    "sr_v_f" :sr_v_f, 
    "sr_v_t" :sr_v_t, 
    "p_value":p_value,
    "i_value":i_value,
    "d_value":d_value
}
file = open('config.conf', 'wb')
pickle.dump(config, file)
file.close()
cv.destroyAllWindows()