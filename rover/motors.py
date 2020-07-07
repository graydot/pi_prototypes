import RPi.GPIO as g
from time import sleep

l=[[27,17],[6,5]]
r=[[23,22],[12,13]]

def get_pins(wheel):
    if wheel == "r":
        in1 = r1
        in2 = r2
    else:
        in1 = l1
        in2 = l2
    return (in1, in2)

def setup(wheels):
    for w in wheels:
        in1,in2 = w
        g.setup(in1, g.OUT)
        g.setup(in2, g.OUT)
        g.output(in1, g.LOW)
        g.output(in2, g.LOW)
    
def forward_w(wheels):
    for w in wheels:
        in1,in2 = w
        g.output(in1, g.LOW)
        g.output(in2, g.HIGH)
    
def stop_w(wheels):
    for w in wheels:
        in1,in2 = w
        g.output(in1, g.LOW)
        g.output(in2, g.LOW)
        
def back_w(wheels):
    for w in wheels:
        in1,in2 = w
        g.output(in1, g.HIGH)
        g.output(in2, g.LOW)
    
def forward(t):
    forward_w(r)
    forward_w(l)
    sleep(t)
    
def back(t):
    back_w(r)
    back_w(l)
    sleep(t)
    
def stop(t):
    stop_w(r)
    stop_w(l)
    sleep(t)
    
def left(t):
    forward_w(r)
    back_w(l)
    sleep(t)

def right(t):
    back_w(r)
    forward_w(l)
    sleep(t)
    
    
g.setmode(g.BCM)
setup(r)
setup(l)
print("forward")
forward(1)
print("stop")
stop(1)
left(1)
stop(1)
print("back")
back(1)

g.cleanup()
