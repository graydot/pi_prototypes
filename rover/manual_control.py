import RPi.GPIO as g
from time import sleep

l=[[27,17],[6,5]]
r=[[23,22],[12,13]]
mode = None

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
    
def forward():
    global mode
    forward_w(r)
    forward_w(l)
    mode = forward
    
def back():
    global mode
    back_w(r)
    back_w(l)
    mode = back
    
def stop():
    stop_w(r)
    stop_w(l)
    
def left():
    global mode
    forward_w(r)
    back_w(l)
    # mode() if mode else stop()


def right():
    global mode
    back_w(r)
    forward_w(l)
    # mode() if mode else stop()
    
g.setmode(g.BCM)
setup(r)
setup(l)

import curses

from curses import wrapper


def main(stdscr):
    # curses.noecho()
    curses.cbreak()
    stdscr.keypad(True) 
    try:
        while True:
            c = stdscr.getch()
            if c == ord('w'):
                forward()
            elif c == ord('q'):
                stop()
                break
            elif c == ord('s'):
                back()
            elif c == ord('x'):
                stop()
            elif c == ord('z'):
                stop()
            elif c == ord('c'):
                stop()
            elif c == ord('a'):
                left()
            elif c == ord('d'):
                right()

    finally:
        print("Called")
        stop()
        g.cleanup()
        curses.nocbreak()
        stdscr.keypad(False)
        curses.echo()


wrapper(main)


