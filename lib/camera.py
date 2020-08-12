from multiprocessing import Queue
from queue import Empty
from PIL import Image
# from picamera.array import PiRGBArray
from picamera import PiCamera, Color
import time
import gphoto2 as gp
import numpy as np
import io
import queue


import sys
sys.path.insert(1, './lib')
from queueoutput import QueueOutput


class ThreadedCamera:
    def __init__(self, 
        mode):
        # Do not intialize the hardware camera in init. Use in start instead as that is threadsafe
        self.output = QueueOutput(
            [queue.SimpleQueue()],
            [])

        self.mode = mode
        self.camera = None
        self.set_resolution()

    def set_resolution(self):
        raise("Override set_resolution()")

    def start(self):
        raise("Override start()")

    def set_overlay_frame(self, frame):
        pass

    def cleanup(self):
        pass
    

class ThreadedPiCamera(ThreadedCamera):
    def set_resolution(self):
        self.resolution = (1280, 720)
                

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

    def create_camera(self):
        camera = PiCamera(resolution=self.resolution)
        time.sleep(1) # camera warmup
        camera.start_preview()
        camera.annotate_foreground = Color(y=0.2,u=0, v=0)
        camera.annotate_background = Color(y=0.8, u=0, v=0)
        self.camera = camera

    def start_video_camera(self):
        self.camera.start_recording(self.output, format="mjpeg")

    def start_photo_camera(self):
        for frame in self.camera.capture_continuous(self.output, format="jpeg"):
            pass
    def start(self):
        try:
            # Have to create camera here for it to work with multiprocessing
            if self.camera is None:
                self.create_camera()
                if self.mode == "video":
                    self.start_video_camera()
                else: 
                    self.start_photo_camera()
            while True:
                time.sleep(0.01)
            # self.start_preview()
        finally:
            if self.mode == "video":
                self.camera.stop_recording()
    def get_frame(self):
        if self.camera is None:
            return None
        try:
            frame = None
            while True:
                frame = self.output.queue.get(False)
        except Empty:
            return frame
    def set_overlay_frame(self, frame):
        if self.camera is None:
            return None
        if self.detector_overlay:
            self.detector_overlay.update(frame)
        else:
            pil_im = Image.frombytes('RGB', self.resolution, frame)
            self.detector_overlay = self.camera.add_overlay(frame, size = pil_im.size, layer = 3)
            self.detector_overlay.layer = 3
            self._monkey_patch_picamera(self.detector_overlay)
    



class ThreadedGPhoto2Camera(ThreadedCamera):
    def set_resolution(self):
        self.resolution = (960,640)
    def create_camera(self):
        try:
            camera = gp.Camera()
            camera.init()
            self.camera = camera
        except gp.GPhoto2Error as ex:
            if ex.code == gp.GP_ERROR_MODEL_NOT_FOUND:
                print("Error: Check if Camera is connected properly")
            else:
                print("Unexpected error:", sys.exc_info())
        except Exception:
            print("Unexpected error:", sys.exc_info())

    def start(self):
        try:
            # Have to create camera here for it to work with multiprocessing
            if self.camera is None:
                self.create_camera()
        except Exception:
            print("Unexpected error:", sys.exc_info())
            
    def _camerafile2frame(self, capture):
        filedata = capture.get_data_and_size()
        return bytes(filedata)

    def get_frame(self):
        if self.camera is None:
            return None
        return self._camerafile2frame(self.camera.capture_preview())

    def save_hires_frame(self, target):
        if self.camera is None:
            return None
        self.camera.exit()
        self.camera.init()
        file_path = self.camera.capture(gp.GP_CAPTURE_IMAGE)
        camera_file = self.camera.file_get(
            file_path.folder, file_path.name, gp.GP_FILE_TYPE_NORMAL)
        camera_file.save(target)
        print("Saved file to", target)
        
        return self._camerafile2frame(self.camera.capture_preview())
    def cleanup(self):
        if self.camera:
            self.camera.exit()




