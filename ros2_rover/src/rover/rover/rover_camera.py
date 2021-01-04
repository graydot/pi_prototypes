from picamera import PiCamera
from rover.image_writer import ImageWriter
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from io import BytesIO
import numpy as np
from datetime import datetime

class RoverCamera(Node):
    def __init__(self):
        super().__init__('rover_camera')
        self.publisher_ = self.create_publisher(CompressedImage, 'raw_images', 10)
        self.camera = PiCamera()
        # self.camera.start_preview()
        self.camera.resolution = (1024, 768)
        self.image_writer = ImageWriter(self, self.publisher_)
        self.start()


    def start(self):
        while True:
            self.camera.start_recording(self.image_writer, 'mjpeg')
            self.camera.wait_recording(2)
            self.camera.stop_recording()
        


def main():
    print('Starting Camera.')
    rclpy.init()
    cam = RoverCamera()
    rclpy.spin(cam)
    cam.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
