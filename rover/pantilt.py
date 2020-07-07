import time
import RPi.GPIO as GPIO
from PCA9685 import PCA9685


# Setup range of allowed motion. Need to figure out a better way to do this
tilt_servo = 0
pan_servo = 1
hrange = [0,180]
vrange = [70,60]
try:
    print ("This is an PCA9685 routine")
    pwm = PCA9685()
    pwm.setPWMFreq(50)
    # pwm.setServoPulse(1,500) 
    pwm.setRotationAngle(1, 0)
    current_h = hrange[0]
    h_dir = 1
    current_v = vrange[0]
    v_dir = 1

    while True:
        # setServoPulse(2,2500)
        current_h += h_dir
        if h_dir == 1 and current_h == hrange[1]:
            h_dir = -1
        if h_dir == -1 and current_h == hrange[0]:
            h_dir = 1

        current_h -= v_dir
        if v_dir == 1 and current_v == vrange[1]:
            v_dir = -1
        if v_dir == -1 and current_v == vrange[0]:
            v_dir = 1

        time.sleep(0.01)

except:
    pwm.exit_PCA9685()
    print ("\nProgram end")
    exit()