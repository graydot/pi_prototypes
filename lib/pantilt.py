from PCA9685 import PCA9685
import logging

class PanTilt:
    def __init__(self,
                 tilt_servo = 0,
                 pan_servo = 1,
                 pan_range = [0,180],
                 tilt_range = [0,180],
                 start_pan = 90,
                 start_tilt = 90
                 ):
        self.tilt_servo = tilt_servo
        self.pan_servo = pan_servo
        # Setup range of allowed motion. Need to figure out a better way to do this
        # FIXME: Autodetect?
        self.pan_range = pan_range
        self.tilt_range = tilt_range
        self.pwm = PCA9685()
        self.pwm.setPWMFreq(50)
        self._pan = start_pan
        self._tilt = start_tilt
        self.pan(start_pan)
        self.tilt(start_tilt)
        
    def pan(self, angle):
        if angle > self.pan_range[0] and angle < self.pan_range[1]:
            self.pwm.setRotationAngle(self.pan_servo, angle)
            self._pan = angle
        else:
            logging.warning("Pan angle %d outside allowed limits", angle)
        
    def get_pan(self):
        return self._pan
        
        
    def tilt(self, angle):
        if angle > self.tilt_range[0] and angle < self.tilt_range[1]:
            self.pwm.setRotationAngle(self.tilt_servo, angle)
            self._tilt = angle
        else:
            logging.warning("Tilt angle %d outside allowed limits", angle)
        
    def get_tilt(self):
        return self._tilt
        
    
    def cleanup(self):
        self.pwm.exit_PCA9685()
    