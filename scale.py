

def scale(val):
    if (val < -45):
        val += 45
        #new range is 0->-135
        left_speed = int((val/135)*100)
        right_speed = -left_speed
    elif (val < 45):
        #new range is -45->45
        left_speed = 0 #
        right_speed = -(int((val/45)*100))
    elif (val <= 180):
        val -= 45
        #new range is 0->135
        left_speed = int((val/135)*100)
        right_speed = 0
    else:
        speed = 0
    return left_speed,right_speed



for even_numbers in range(-180,180,2):
    speed = scale(even_numbers)
    print(even_numbers," scales to ", speed)

# this won't work because, as the angle gets closer to zero it will crab-walk / oscillate back and forth towards the target
# speed should be an adjustment to an existing speed. Try using PID controller. eg. https://pypi.org/project/simple-pid/

# Try: 
# IF target found:
# outside wheel speed = max.
# Inside wheel speed in range -100% -> 100%
# ELSE: #target not found
# both wheels = 0