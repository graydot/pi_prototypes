# -*- coding: utf-8 -*-
import cv2
from picamera import PiCamera, Color
import picamera # for monkey patching
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from io import BytesIO
from multiprocessing import Queue
from queue import Empty
import time
import datetime as dt
import logging
from queueoutput import QueueOutput
import os


class Detector:        
    TENSOR_RESOLUTION = [320,320]
    def __init__(
            self,
            name,
            mode,
            resolution,
            image_queue, image_finished,
            streaming_queue, streaming_finished,
            center_x, center_y,
            labels = ['face']
            ):

        self.resolution = resolution
        self.name = name
        self.mode="video"
        
        self.output = QueueOutput(
            [image_queue],
            [image_finished, streaming_finished])
            
        self.queue = image_queue
        self.streaming_queue = streaming_queue
        self.finished = image_finished
        self.center_x = center_x
        self.center_y = center_y
        
        self.detector_overlay = None
        self.labels = labels
        self.camera = None

    def create_camera(self):
        camera = PiCamera(resolution=self.resolution)
        camera.start_preview()
        camera.annotate_foreground = Color(y=0.2,u=0, v=0)
        camera.annotate_background = Color(y=0.8, u=0, v=0)
        self.camera = camera

    def start_video_camera(self):
        self.camera.start_recording(self.output, format="mjpeg")

    def start_photo_camera(self):
        self.camera.capture_continuous(self.output, format="jpeg")

        
    def start(self):
        try:
            # Have to import it here for it to work with multiprocessing
            if self.camera is None:
                self.create_camera()
                if self.mode == "video":
                    self.start_video_camera()
                else: 
                    self.start_photo_camera()

            if self.labels == ['face']:
                from facessd_mobilenet_v2 import FaceSSD_MobileNet_V2_EdgeTPU
                self.model = FaceSSD_MobileNet_V2_EdgeTPU()
            else:
                from ssd_mobilenet_v3_coco import SSDMobileNet_V3_Coco_EdgeTPU_Quant
                self.model = SSDMobileNet_V3_Coco_EdgeTPU_Quant()
                
                if self.labels is None:
                    from ssd_mobilenet_v3_coco import LABELS as all_coco_labels
                    self.labels = all_coco_labels


            self.label_idxs = self.model.label_to_category_index(self.labels)
            
            output = self.output
            start_time = time.time()
            frame_count = 0
            fps = 0
            
            while not self.finished.wait(0.1):
                try:
                    frame = None
                    while True:
                        frame = self.queue.get(False)
                except Empty:
                    if frame == None:
                        continue 
                    cv2.CV_LOAD_IMAGE_COLOR = 1 # set flag to 1 to give colour image
                    npframe = np.fromstring(frame, dtype=np.uint8)
                    pil_frame = cv2.imdecode(npframe,cv2.CV_LOAD_IMAGE_COLOR)
                    cv2_im_rgb = cv2.cvtColor(pil_frame, cv2.COLOR_BGR2RGB)
                    pil_im = Image.fromarray(cv2_im_rgb)
                    pil_im.save("something.jpg", "JPEG")
                    frame_snapshot = pil_im.copy()
                    tensor_im = pil_im.copy()
                    tensor_im = tensor_im.resize((320,320))
                    tensor_im.save("small.jpg", "JPEG")
                    predictions = self.model.predict(np.array(tensor_im))
                    boxes = predictions.get('detection_boxes')

                    objects = None
                    if len(boxes):
                        classes = predictions.get('detection_classes')
                        scores = predictions.get('detection_scores')
                        objects = filter(lambda item: item in self.label_idxs, classes)
                        def item_to_hashmap(index_and_item):
                            index, item = index_and_item
                            return {
                                "i": index,
                                "name" : self.model.category_index[item]['name'],
                                "box" : boxes[index],
                                "score" : scores[index]
                            }
                        objects = list(map(item_to_hashmap , enumerate(objects)))
                        objects = list(filter(lambda item: item["score"] > 0.5, objects))
        
                        
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

                    # we will send the color to green later so the first object is red
                    thickness = 2
                    color = 'red'
                    tracked_object_name = None
                    for object in objects:
                        (y1, x1, y2, x2) = object['box']
                        (l, r, t, b) = (x1 * im_width, x2 * im_width, y1 * im_height, y2 * im_height)
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
                        if tracked_object_name is None:
                            tracked_object_name = object['name']
                        thickness = 1
                        color = 'LawnGreen'

            
                    font = ImageFont.truetype("/usr/share/fonts/truetype/freefont/FreeSans.ttf", 8)
                    myText = self.name + dt.datetime.now().strftime('%Y-%m-%d %H:%M:%S') +\
                        " FPS " + str(fps) + \
                        (". Tracking " + tracked_object_name if tracked_object_name else "")
        
                    # Draw the text
                    # ***************** if tracking change color to something else
                    if len(objects) > 0:
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
                    self.streaming_queue.put(pil_im.tobytes())
                    if self.detector_overlay:
                        self.detector_overlay.update(pil_im.tobytes())
                    else:
                        self.detector_overlay = self.camera.add_overlay(pil_im.tobytes(), size = pil_im.size, layer = 3)
                        self.detector_overlay.layer = 3
                        _monkey_patch_picamera(self.detector_overlay)

                    self.handle_detect(frame_snapshot, objects)
        finally:
            self.camera.stop_recording()
    def handle_detect(self, frame):
        raise("Do not use the base class, use one of the child classes BirdDetector or FaceDetector")

class BirdDetector(Detector):
    def __init__(self, *args):
        # Create Right folders
        self.ts = None
        # Take current directory and rename to previous one by finding timestamp file in it
        execution_path = os.getcwd()
        self.directory = os.path.join(execution_path, "Images_" + time.strftime("%a_%b_%d_%H_%M_%S_%Z",time.localtime()))
        os.mkdir(self.directory)
        self.file_number = 0
        # call super
        super().__init__(*args)
    def handle_detect(self, frame, objects):
        if len(objects) == 0:
            return
        now = time.time()
        if self.ts is None:
            self.ts = now
        elif now - self.ts < 5:
            # less than 10 secs, ignore
            return
        elif now - self.ts > 5:
            # Handle image and send ts to current time
            pass
        file_name = os.path.join(self.directory, "Bird_" + str(self.file_number) + ".jpg")
        frame.save(file_name, 'jpeg')
        print('.')

        self.file_number += 1
        self.ts = now

class FaceDetector(Detector):
    def handle_detect(self, frame, objects):
        # Optimization. Run Face Detection only once per runModelPerIterations.
        # But process existing bounding boxes every iteration
        if len(objects)>0:
            (y1, x1, y2, x2) = objects[0]['box']
            self.center_x.value = (x1 + x2)/2
            self.center_y.value = (y1 + y2)/2
        else:
            # FIXME - incorporate searching
            # FIXME - do not reset to 0.5 but stop panning and tilting instead
            self.center_x.value = 0.5
            self.center_y.value = 0.5
