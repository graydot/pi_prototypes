# import the necessary packages
import picamera
from picamera.array import PiRGBArray
from picamera import PiCamera
from picamera import Color
from PIL import ImageFont, ImageDraw, Image

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
model = FaceSSD_MobileNet_V2()
if isinstance(model, FaceSSD_MobileNet_V2):
    labels = ['face']
else:
    labels = LABELS
label_idxs = model.label_to_category_index(labels)

rawCapture = PiRGBArray(camera, size=RESOLUTION)

# This stuff is horrible but i am just trying to get this to work now. Need to refactor to classes soon.
import time
import sys
import RPi.GPIO as GPIO
from PCA9685 import PCA9685


# Setup range of allowed motion. Need to figure out a better way to do this
tilt_servo = 0
pan_servo = 1
hrange = [10,170]
vrange = [30,150]
currentPanAngle = 90
currentTiltAngle = 90

pwm = PCA9685()
pwm.setPWMFreq(50)
# pwm.setServoPulse(1,500)
def setPan(angle):
    if angle > hrange[0] and angle < hrange[1]:
        pwm.setRotationAngle(pan_servo, angle)
        currentPanAngle = angle

def getPan():
    return currentPanAngle

def setTilt(angle):
    if angle > vrange[0] and angle < vrange[1]:
        pwm.setRotationAngle(tilt_servo, angle)
        currentTiltAngle = angle

def getTilt():
    return currentTiltAngle

pan_pid = PIDController(0.05, 0.1, 0)
pan_pid.reset()
tilt_pid = PIDController(0.05, 0.2, 0)
tilt_pid.reset()
setPan(90)
setTilt(90)



print("Starting Scan Loop **********")

start_time = time.time()
fps = 0
frame_count = 0
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    # grab the raw NumPy array representing the image, then initialize the timestamp
    # and occupied/unoccupied text
    image = frame.array
    tensor_im = image.copy()
    tensor_im = Image.fromarray(np.uint8(image)).convert('RGB')
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

    if tracked_object:
        (y1, x1, y2, x2) = tracked_object['box']
        tilt_error = 100 - (y1+y2)*200/2
        tilt_value = tilt_pid.update(tilt_error)
        pan_error = 100 - (x1+x2)*200/2
        pan_value = pan_pid.update(pan_error)
        print("******")
        print("******")
        print(tracked_object['box'])
        print("(", x1*200, y1*200, ")", "(", x2*200, y2*200, ")")
        print("Target center is", (x1+x2)*100, (y1 + y2)*100)
        print(pan_error, tilt_error)
        print("pan: ", pan_value)
        print("tilt: ", tilt_value)
        setPan(getPan() + pan_value)
        setTilt(getTilt() - tilt_value)


#     Annotate image for display
#     cv2.CV_LOAD_IMAGE_COLOR = 1 # set flag to 1 to give colour image
#     npframe = np.fromstring(image, dtype=np.uint8)
#     pil_frame = cv2.imdecode(npframe,cv2.CV_LOAD_IMAGE_COLOR)
#     pil_frame = cv2.imdecode(frame,-1)
#     print(pil_frame)
#     cv2_im_rgb = cv2.cvtColor(pil_frame, cv2.COLOR_BGR2RGB)
#     pil_im = Image.fromarray(cv2_im_rgb)

#     draw = ImageDraw.Draw(pil_im)
    pil_im = Image.fromarray(np.uint8(image)).convert('RGB')
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



    rawCapture.truncate(0)


