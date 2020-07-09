import time
import sys
import RPi.GPIO as GPIO
from PCA9685 import PCA9685


# Setup range of allowed motion. Need to figure out a better way to do this
tilt_servo = 0
pan_servo = 1
hrange = [0,180]
vrange = [0,180]

#=====

#======
pwm = PCA9685()
pwm.setPWMFreq(50)
# pwm.setServoPulse(1,500)
pwm.setRotationAngle(1, 90)
    current_h = hrange[0]
    h_dir = 1
    current_v = vrange[0]
    v_dir = 2

    while True:
        # setServoPulse(2,2500)
        print str(current_h)
        print str(current_v)
        pwm.setRotationAngle(1,current_h)
        pwm.setRotationAngle(0,current_v)
        current_h += h_dir
        if (h_dir > 0 and current_h == hrange[1]) or \
            (h_dir < 0 and current_h == hrange[0]):
            h_dir *= -1

        current_v += v_dir
        if (v_dir > 0 and current_v == vrange[1]) or \
            (v_dir < 0 and current_v == vrange[0]):
            v_dir *= -1



        time.sleep(0.01)

except:
    pwm.exit_PCA9685()
    print ("\nProgram end", sys.exc_info()[0], sys.exc_info()[1])
    exit()