# import the necessary packages
import picamera
from picamera.array import PiRGBArray
from picamera import PiCamera
from picamera import Color
from PIL import ImageFont, ImageDraw, Image
from threading import Condition, Thread
from queueoutput import QueueOutput
from pivideoserver import PiVideoServer



import numpy as np
import time
import datetime as dt
import cv2
import os
import io
import logging
from pid import PIDController
from ssd_mobilenet_v3_coco import SSDMobileNet_V3_Small_Coco_PostProcessed as SSMobileNetV3
from facessd_mobilenet_v2 import FaceSSD_MobileNet_V2
from ssd_mobilenet_v3_coco import LABELS
import traceback

import RPi.GPIO as g
from time import sleep
from drive import Drive
from pantilt import PanTilt
from pivideoserver import PiVideoServer
from detector import Detector
from multiprocessing import Queue, Process, Value, Event, Manager, Pipe
import signal
import sys



enable_pantilt = False
enable_drive = False

# Credits to https://github.com/leigh-johnson/rpi-deep-pantilt
# ********************* Fix Resolution ***********
RESOLUTION = (640,480)
START_PAN = 90
START_TILT = 45
GRID_SCALE = 200 # We don't use resolutions in our code but an arbitrary dimension
ERRVAL_IGNORED = 10 # in percentage

LWHEELS = [[27,17],[6,5]]
RWHEELS = [[23,22],[12,13]]
global drive
drive = None


def signal_handler(sig, frame):
    # print a status message
    print("[INFO] You pressed `ctrl + c`! Exiting...")

    # disable the servos
    if drive:
        drive.stop()
    sys.exit()

def pid_process(output, p, i, d, box_coord, origin_coord, action):
    # signal trap to handle keyboard interrupt
    signal.signal(signal.SIGINT, signal_handler)

    # create a PID and initialize it
    p = PIDController(p.value, i.value, d.value)
    p.reset()

    # loop indefinitely
    while True:
        error = (origin_coord - box_coord.value) * GRID_SCALE
        if abs(error/GRID_SCALE) * 100 < ERRVAL_IGNORED: #error is < 10%. Ignore
            error = 0
            #p.reset()
        output.value = p.update(error)

def set_servos(servos, pan, tilt):
    # signal trap to handle keyboard interrupt
    signal.signal(signal.SIGINT, signal_handler)
    

    while True:
        pan_angle = 90 + pan.value
        tilt_angle = 90 - tilt.value #need to swap directions

        # if the pan angle is within the range, pan
        
        servos.pan(pan_angle)  
        servos.tilt(tilt_angle)

def set_turn(drive, turn):
    # signal trap to handle keyboard interrupt
    signal.signal(signal.SIGINT, signal_handler)
    
    # I am reusing angle returned from pid controll
    while True:
        if turn.value < 0:
            drive.left(0.1)
        elif turn.value > 0:
            drive.right(0.1)
        else:
            drive.stop()
        sleep(0.1)

with Manager() as manager:
    image_queue = manager.Queue()
    image_finished = manager.Event()
    streaming_queue = manager.Queue()
    streaming_finished = manager.Event()
    

    
    center_x = manager.Value('i', 0.5)
    center_y = manager.Value('i', 0.5)
   

    # pan and tilt angles updated by independent PID processes
    pan = manager.Value('i', 0)
    tilt = manager.Value('i', 0)
    
    turn = manager.Value('i', 0)

    # PID gains for panning

    pan_p = manager.Value('f', 0.05)
    pan_i = manager.Value('f', 0.1)
    pan_d = manager.Value('f', 0)

    # PID gains for tilting
    tilt_p = manager.Value('f', 0.15)
    tilt_i = manager.Value('f', 0.1)
    tilt_d = manager.Value('f', 0)
    
    # PID gains for drive turn
    turn_p = manager.Value('f', 0.5)
    turn_i = manager.Value('f', 0.5)
    turn_d = manager.Value('f', 0.5)
    
    detector = Detector(
        RESOLUTION, 
        image_queue, image_finished,
        streaming_queue, streaming_finished,
        center_x, center_y)
    servos = PanTilt(pan_range=[10, 170], tilt_range=[30,150], start_pan = START_PAN, start_tilt = START_TILT)
    drive = Drive(LWHEELS, RWHEELS)
    video = PiVideoServer(streaming_queue, streaming_finished, RESOLUTION)



    pan_process = Process(target=pid_process,
                          args=(pan, pan_p, pan_i, pan_d, center_x, 0.5, 'pan'))

    tilt_process = Process(target=pid_process,
                           args=(tilt, tilt_p, tilt_i, tilt_d, center_y, 0.5, 'tilt'))
    
    turn_process = Process(target=pid_process,
                           args=(turn, turn_p, turn_i, turn_d, pan, 0, 'turn'))
    
    
    detector_process = Process(target=detector.start)
    servo_process = Process(target=set_servos, args=(servos, pan, tilt))
    drive_process = Process(target=set_turn, args=(drive, turn))
    video_process = Process(target=video.start)
    procs = [
        servo_process,
        pan_process,
        tilt_process,
        turn_process,
        drive_process,
        detector_process, 
        video_process
        ]
    for proc in procs:
        proc.start()
    for proc in procs:
        proc.join()



