import serial
import time

robot=serial.Serial('/dev/ttyS2', 9600)
time.sleep(2)

left_motorspeed_mask = 0b10000000
right_motorspeed_mask = 0b11000000

def send_speed(var):
    # Receive an tuple (left motor, right motor). The value may not be an integer,
    # so cast it. The value must be shifted to fit in the available bit-space (6 bits).
    # Add 32, to shift the values from -31-31 to 0-63.
    # The individual motor speeds must be sent sepearately, and must 
    # incorporate some bit flags to identify the command. Set the MSB to 1 to indicate
    # motor speed commands. Set the second MSB (7) to 0 for the left motor, and to
    # 1 for the right motor. Then combine with the motor speeds and send. 
    left_speed = int(var[0])
    left_speed = left_speed | left_motorspeed_mask
    write = robot.write(bytes(chr(left_speed),'utf-8'))
    if (write != 1):
        print ("LEFT Bytes sent: ", write)
    right_speed = int(var[1])
    right_speed = right_speed | right_motorspeed_mask
    write = robot.write(bytes(chr(right_speed),'utf-8'))
    if (write != 1):
        print ("RIGHT Bytes sent: ", write)
    return

def read():
    return robot.read_all()

