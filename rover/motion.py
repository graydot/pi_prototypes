# import the necessary packages
import picamera
from picamera.array import PiRGBArray
from picamera import PiCamera
from picamera import Color
from PIL import ImageFont, ImageDraw, Image
from threading import Condition, Thread
import socket
import socketserver
from http.server import BaseHTTPRequestHandler,HTTPServer

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
from facessd_mobilenet_v2 import FaceSSD_MobileNet_V2_EdgeTPU
from ssd_mobilenet_v3_coco import LABELS
import traceback

import RPi.GPIO as g
from time import sleep

enable_pantilt = False
enable_drive = False

lwheels=[[27,17],[6,5]]
rwheels=[[23,22],[12,13]]

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
    forward_w(rwheels)
    forward_w(lwheels)
    sleep(t)
    stop()

def back(t):
    back_w(rwheels)
    back_w(lwheels)
    sleep(t)
    stop()

def stop():
    stop_w(rwheels)
    stop_w(lwheels)

def left(t):
    forward_w(rwheels)
    back_w(lwheels)
    sleep(t)
#     stop()


def right(t):
    back_w(rwheels)
    forward_w(lwheels)
    sleep(t)
#     stop()



# 1296x730
# Credits to https://github.com/leigh-johnson/rpi-deep-pantilt
# ********************* Fix Resolution ***********
RESOLUTION = (640,480)

camera = PiCamera(resolution=RESOLUTION, framerate=24)
camera.annotate_foreground = Color(y=0.2,u=0, v=0)
camera.annotate_background = Color(y=0.8, u=0, v=0)
# camera.start_preview()
my_overlay = None
# model = SSMobileNetV3()
model = FaceSSD_MobileNet_V2_EdgeTPU()
if isinstance(model, FaceSSD_MobileNet_V2_EdgeTPU):
    labels = ['face']
else:
    labels = ['person']
label_idxs = model.label_to_category_index(labels)

rawCapture = PiRGBArray(camera, size=RESOLUTION)

# This stuff is horrible but i am just trying to get this to work now. Need to refactor to classes soon.
from PCA9685 import PCA9685


# Setup range of allowed motion. Need to figure out a better way to do this
tilt_servo = 0
pan_servo = 1
hrange = [10,170]
vrange = [30,150]
currentPanAngle = 90
currentTiltAngle = 45

pwm = PCA9685()
pwm.setPWMFreq(50)
# pwm.setServoPulse(1,500)
def setPan(angle):
    global currentPanAngle
    if angle > hrange[0] and angle < hrange[1]:
        pwm.setRotationAngle(pan_servo, angle)
        currentPanAngle = angle

def getPan():
    return currentPanAngle

def setTilt(angle):
    global currentTiltAngle
    if angle > vrange[0] and angle < vrange[1]:
        pwm.setRotationAngle(tilt_servo, angle)
        currentTiltAngle = angle

def getTilt():
    return currentTiltAngle

g.setmode(g.BCM)
setup(rwheels)
setup(lwheels)




print("Starting Scan Loop **********")

class StreamingOutput(object):
    def __init__(self):
        self.frame = None
        self.buffer = io.BytesIO()
        self.condition = Condition()

    def write(self, buf):
        if buf.startswith(b'\xff\xd8'):
            # New frame, copy the existing buffer's content and notify all
            # clients it's available
            self.buffer.truncate()
            with self.condition:
                self.frame = self.buffer.getvalue()
                self.condition.notify_all()
            self.buffer.seek(0)
        return self.buffer.write(buf)


start_time = time.time()
fps = 0
frame_count = 0
output = StreamingOutput()

class StreamingHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self.send_response(301)
            self.send_header('Location', '/index.html')
            self.end_headers()
        elif self.path == '/index.html':
            content = PAGE.encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.send_header('Content-Length', len(content))
            self.end_headers()
            self.wfile.write(content)
        elif self.path == '/stream.mjpg':
            self.send_response(200)
            self.send_header('Age', 0)
            self.send_header('Cache-Control', 'no-cache, private')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
            self.end_headers()
            try:
                while True:
                    with output.condition:
                        output.condition.wait()
                        frame = output.frame
                        # now add timestamp to jpeg
                        # Convert to PIL Image
                        cv2.CV_LOAD_IMAGE_COLOR = 1 # set flag to 1 to give colour image
                        npframe = np.fromstring(frame, dtype=np.uint8)
                        pil_frame = cv2.imdecode(npframe,cv2.CV_LOAD_IMAGE_COLOR)
                        #pil_frame = cv2.imdecode(frame,-1)
                        cv2_im_rgb = cv2.cvtColor(pil_frame, cv2.COLOR_BGR2RGB)
                        pil_im = Image.fromarray(cv2_im_rgb)

                        draw = ImageDraw.Draw(pil_im)

                        # Choose a font
                        font = ImageFont.truetype("/usr/share/fonts/truetype/freefont/FreeSans.ttf", 25)
                        myText = "Rover Eyes "+dt.datetime.now().strftime('%Y-%m-%d %H:%M:%S')

                        # Draw the text
                        color = 'rgb(255,255,255)'
                        #draw.text((0, 0), myText,fill = color, font=font)

                        # get text size
                        text_size = font.getsize(myText)

                        # set button size + 10px margins
                        button_size = (text_size[0]+20, text_size[1]+10)

                        # create image with correct size and black background
                        button_img = Image.new('RGBA', button_size, "black")

                        #button_img.putalpha(128)
                        # put text on button with 10px margins
                        button_draw = ImageDraw.Draw(button_img)
                        button_draw.text((10, 5), myText, fill = color, font=font)

                        # put button on source image in position (0, 0)

                        pil_im.paste(button_img, (0, 0))
                        bg_w, bg_h = pil_im.size
                        # WeatherSTEM logo in lower left
                        size = 64
                        # WSLimg = Image.open("WeatherSTEMLogoSkyBackground.png")
                        # WSLimg.thumbnail((size,size),Image.ANTIALIAS)
                        # pil_im.paste(WSLimg, (0, bg_h-size))

                        # SkyWeather log in lower right
                        # SWLimg = Image.open("SkyWeatherLogoSymbol.png")
                        # SWLimg.thumbnail((size,size),Image.ANTIALIAS)
                        # pil_im.paste(SWLimg, (bg_w-size, bg_h-size))

                        # Save the image
                        buf= io.StringIO()
                        pil_im.save(buf, format= 'JPEG')
                        frame = buf.getvalue()
                    self.wfile.write(b'--FRAME\r\n')
                    self.send_header('Content-Type', 'image/jpeg')
                    self.send_header('Content-Length', len(frame))
                    self.end_headers()
                    self.wfile.write(frame)
                    self.wfile.write(b'\r\n')
            except Exception as e:
                traceback.print_exc()
                logging.warning(
                    'Removed streaming client %s: %s',
                    self.client_address, str(e))
        else:
            self.send_error(404)
            self.end_headers()

class StreamingServer(socketserver.ThreadingMixIn, HTTPServer):
    allow_reuse_address = True
    daemon_threads = True
    
def start_server():
    address = ('', 1025)
    print("Address is http://" + socket.getfqdn() + ":" + str(address[1]))
    server = StreamingServer(address, StreamingHandler)
    server.serve_forever()
    print("Serving at " + address)


# I am using start_recording instead of capture continuous as the latter was choppy
camera.start_recording(output, format="mjpeg")

server = Thread(target=start_server)
server.start()


time.sleep(1)

pan_pid = PIDController(0.05, 0.1, 0)
pan_pid.reset()
tilt_pid = PIDController(0.05, 0.1, 0)
tilt_pid.reset()
drive_pid = PIDController(0.05, 0.05, 0)
drive_pid.reset()
setPan(currentPanAngle)
setTilt(currentTiltAngle)
runModelPerIterations = 1
currentIteration = 0
tracked_object = None
try:

    while True:

        with output.condition:
            output.condition.wait()
            frame = output.frame


        cv2.CV_LOAD_IMAGE_COLOR = 1 # set flag to 1 to give colour image
        npframe = np.fromstring(frame, dtype=np.uint8)
        pil_frame = cv2.imdecode(npframe,cv2.CV_LOAD_IMAGE_COLOR)
        #pil_frame = cv2.imdecode(frame,-1)
        cv2_im_rgb = cv2.cvtColor(pil_frame, cv2.COLOR_BGR2RGB)
        pil_im = Image.fromarray(cv2_im_rgb)

        if currentIteration % runModelPerIterations == 0:
            tensor_im = pil_im.copy()
            tensor_im = tensor_im.resize((320,320))

            predictions = model.predict(np.array(tensor_im))

            boxes = predictions.get('detection_boxes')
            if len(boxes):
                classes = predictions.get('detection_classes')
                scores = predictions.get('detection_scores')
                objects = filter(lambda item: item in label_idxs, classes)
                def item_to_hashmap(index_and_item):
                    index, item = index_and_item
                    return {
                        "i": index,
                        "name" : model.category_index[item]['name'],
                        "box" : boxes[index],
                        "score" : scores[index]
                    }
                objects = list(map(item_to_hashmap , enumerate(objects)))
                objects = list(filter(lambda item: item["score"] > 0.5, objects))

                if len(objects) > 0:
                    tracked_object = objects[0]
                else:
                    tracked_object = None
            else:
                tracked_object = None

            #calculate FPS
            current_time = time.time()
            time_elapsed = current_time - start_time
            if time_elapsed >= 1:
                fps = int(frame_count/time_elapsed)
                start_time = current_time
                frame_count = 0
            frame_count += 1

            draw = ImageDraw.Draw(pil_im)
            (im_width, im_height) = pil_im.size

            for object in objects:
                (y1, x1, y2, x2) = object['box']
                (l, r, t, b) = (x1 * im_width, x2 * im_width, y1 * im_height, y2 * im_height)
                if object == tracked_object:
                    thickness = 2
                    color = 'red'
                else:
                    thickness = 1
                    color = 'LawnGreen'
                draw.line([(l, t), (l, b), (r, b), (r, t), (l, t)],\
                    width = thickness, fill = color)

                try:
                    font = ImageFont.truetype('arial.ttf', 24)
                except IOError:
                    font = ImageFont.load_default()

                display_str = object['name'] + ", " + str(object["score"])
                # If the total height of the display strings added to the top of the bounding
                # box exceeds the top of the image, stack the strings below the bounding box
                # instead of above.
                display_str_height = font.getsize(display_str)[1]
                # Each display_str has a top and bottom margin of 0.05x.
                total_display_str_height = (1 + 2 * 0.05) * display_str_height

                if t > total_display_str_height:
                    text_bottom = t
                else:
                    text_bottom = b + total_display_str_height
                text_width, text_height = font.getsize(display_str)
                margin = np.ceil(0.05 * text_height)
                draw.rectangle(
                    [(l, text_bottom - text_height - 2 * margin), (l + text_width,
                                                                      text_bottom)],
                    fill=color)
                draw.text(
                    (l + margin, text_bottom - text_height - margin),
                    display_str,
                    fill='black',
                    font=font)
                text_bottom -= text_height - 2 * margin

            font = ImageFont.truetype("/usr/share/fonts/truetype/freefont/FreeSans.ttf", 8)
            myText = "Rover Eyes " + dt.datetime.now().strftime('%Y-%m-%d %H:%M:%S') +\
                " FPS " + str(fps) + \
                (". Tracking " + tracked_object["name"] if tracked_object else "")
            print (myText)

            # Draw the text
            # ***************** if tracking change color to something else
            if tracked_object:
                color = 'rgb(255, 0, 0)'
            else:
                color = 'rgb(255,255,255)'
            #draw.text((0, 0), myText,fill = color, font=font)

            # get text size
            text_size = font.getsize(myText)

            # set button size + 10px margins
            button_size = (text_size[0]+20, text_size[1]+10)

            # create image with correct size and black background
            button_img = Image.new('RGBA', button_size, "black")

            #button_img.putalpha(128)
            # put text on button with 10px margins
            button_draw = ImageDraw.Draw(button_img)
            button_draw.text((10, 5), myText, fill = color, font=font)

            # put button on source image in position (0, 0)

            pil_im.paste(button_img, (0, 0))
            bg_w, bg_h = pil_im.size
            # WeatherSTEM logo in lower left
            size = 64
            # WSLimg = Image.open("WeatherSTEMLogoSkyBackground.png")
            # WSLimg.thumbnail((size,size),Image.ANTIALIAS)
            # pil_im.paste(WSLimg, (0, bg_h-size))

            # SkyWeather log in lower right
            # SWLimg = Image.open("SkyWeatherLogoSymbol.png")
            # SWLimg.thumbnail((size,size),Image.ANTIALIAS)
            # pil_im.paste(SWLimg, (bg_w-size, bg_h-size))

            # Save the image
            def _monkey_patch_picamera(overlay):
                original_send_buffer = picamera.mmalobj.MMALPortPool.send_buffer

                def silent_send_buffer(zelf, *args, **kwargs):
                    try:
                        original_send_buffer(zelf, *args, **kwargs)
                    except picamera.exc.PiCameraMMALError as error:
                        # Only silence MMAL_EAGAIN for our target instance.
                        our_target = overlay.renderer.inputs[0].pool == zelf
                        if not our_target or error.status != 14:
                            raise error

                picamera.mmalobj.MMALPortPool.send_buffer = silent_send_buffer

            if my_overlay:
                my_overlay.update(pil_im.tobytes())
            else:
                my_overlay = camera.add_overlay(pil_im.tobytes(), size = pil_im.size, layer = 3)
                my_overlay.layer = 3
                _monkey_patch_picamera(my_overlay)

        # Optimization. Run Face Detection only once per runModelPerIterations.
        # But process existing bounding boxes every iteration
        if tracked_object:
                (y1, x1, y2, x2) = tracked_object['box']
                print("Object is in", tracked_object['box'])

                if enable_pantilt:
                    tilt_error = 100 - (y1+y2)*200/2
                    if abs(tilt_error) < 200/20:
                            tilt_error = 0
                    tilt_value = int(tilt_pid.update(tilt_error))
                    pan_error = 100 - (x1+x2)*200/2
                    if abs(pan_error) < 200/20:
                        pan_error = 0
                    pan_value = int(pan_pid.update(pan_error))
                    print("Delta required is", pan_error, tilt_error)
                    print("Commands are", pan_value, tilt_value)
                    setTilt(90 - tilt_value)
                    setPan(90 + pan_value)
                if enable_drive:
                    drive_error = 90 - getPan()
                    if (abs(drive_error) < 10):
                        drive_error = 0
                    drive_value = drive_pid.update(drive_error)/10

                    if abs(drive_value) > 10:
                        raise "Risk of Spinning"

                    if (drive_value ==0 or drive_error ==0):
                        stop()
                    elif (drive_value > 0):
                        right(abs(drive_value))
                    elif (drive_value < 0):
                        left(abs(drive_value))
        else:
            if enable_drive:
                stop()


        currentIteration += 1
        print(currentIteration)


        rawCapture.truncate(0)
finally:
    stop()
    camera.stop_recording()

