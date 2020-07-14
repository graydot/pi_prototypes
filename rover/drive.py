# -*- coding: utf-8 -*-

import RPi.GPIO as g
from time import sleep

class Drive:
    def __init__(self, 
                 left_wheels,
                 right_wheels
    ):
        self.lwheels = left_wheels
        self.rwheels = right_wheels
            
        g.setmode(g.BCM)
        self.setup(left_wheels)
        self.setup(right_wheels)

    
    def setup(self, wheels):
        for w in wheels:
            in1,in2 = w
            g.setup(in1, g.OUT)
            g.setup(in2, g.OUT)
            g.output(in1, g.LOW)
            g.output(in2, g.LOW)
    
    def forward_w(self, wheels):
        for w in wheels:
            in1,in2 = w
            g.output(in1, g.LOW)
            g.output(in2, g.HIGH)
    
    def stop_w(self, wheels):
        for w in wheels:
            in1,in2 = w
            g.output(in1, g.LOW)
            g.output(in2, g.LOW)
    
    def back_w(self, wheels):
        for w in wheels:
            in1,in2 = w
            g.output(in1, g.HIGH)
            g.output(in2, g.LOW)
    
    def forward(self, t):
        self.forward_w(self.rwheels)
        self.forward_w(self.lwheels)
        sleep(t)
        self.stop()
    
    def back(self, t):
        self.back_w(self.rwheels)
        self.back_w(self.lwheels)
        self.sleep(t)
        self.stop()
    
    def stop(self):
        self.stop_w(self.rwheels)
        self.stop_w(self.lwheels)
    
    def left(self, t):
        self.forward_w(self.rwheels)
        self.back_w(self.lwheels)
        sleep(t)
        self.stop()
    
    
    def right(self, t):
        self.back_w(self.rwheels)
        self.forward_w(self.lwheels)
        sleep(t)
        self.stop()
