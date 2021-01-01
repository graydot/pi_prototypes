from picamera import PiCamera
from rover.image_writer import ImageWriter
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from io import BytesIO
import numpy as np

class RoverCamera(Node):
    def __init__(self):
        super().__init__('rover_camera')
        self.publisher_ = self.create_publisher(CompressedImage, 'raw_images', 10)
        self.camera = PiCamera()
        self.camera.start_preview()

        self.image_writer = ImageWriter(self.publisher_)
        self.start()


    def start(self):
        stream = BytesIO()  
        i = 0
        for foo in self.camera.capture_continuous(stream, format='jpeg'):  
            # Truncate the stream to the current position (in case  
            # prior iterations output a longer image)  
            stream.truncate()  
            stream.seek(0)
            image_bytes = stream.getvalue()
            print("Wrote")
            msg = CompressedImage()
            
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.format = "jpeg"
            msg.data = image_bytes
            self.publisher_.publish(msg)



def main():
    print('Hi from rover.')
    rclpy.init()
    cam = RoverCamera()
    rclpy.spin(cam)
    cam.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
