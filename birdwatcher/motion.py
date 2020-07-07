# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import os
from imageai.Detection import ObjectDetection
import os

min_area = 20000
max_area = 200000
previous_time = None 
firstFrame = None
execution_path = os.getcwd()
directory = os.path.join(execution_path, "images_" + str(int(time.time())))
os.mkdir(directory)
# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))
# allow the camera to warmup
time.sleep(0.1)
# capture frames from the camera
print("Initializing **********")
detector = ObjectDetection()
detector.setModelTypeAsYOLOv3()
detector.setModelPath( os.path.join(execution_path , "yolo.h5"))
detector.loadModel()
custom_objects = detector.CustomObjects(bird=True, person=True)
print("Starting Scan Loop **********")
for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    # grab the raw NumPy array representing the image, then initialize the timestamp
    # and occupied/unoccupied text
    image = frame.array
    print("------", str(time.time()))

    run_time = time.time()
    run_time_str = str(run_time)
    debug_file_name = os.path.join(directory, "debug_" + run_time_str + ".jpg")
    detections = detector.detectCustomObjectsFromImage(
        input_type="array",
        custom_objects=custom_objects, 
        input_image=image, 
        output_image_path=debug_file_name, 
        minimum_percentage_probability=50, 
        display_percentage_probability=False)
    print("Detect took ", str(time.time()-run_time), " seconds")
    for eachObject in detections:
        print(eachObject["name"] , " detected")
    if len(detections) > 0:
        cv2.imwrite(
            os.path.join(directory, "bird_" + run_time_str + ".jpg"), 
            image)
        # time.sleep(1)
    else:
        os.remove(debug_file_name)
    rawCapture.truncate(0)



