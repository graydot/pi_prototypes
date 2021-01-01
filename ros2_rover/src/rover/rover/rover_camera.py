from picamera import PiCamera
from rover.image_writer import ImageWriter
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class RoverCamera(Node):
    def __init__(self):
        super().__init__('rover_camera')
        self.publisher_ = self.create_publisher(String, 'raw_images', 10)
        self.camera = PiCamera()
        self.camera.start_preview()

        self.image_writer = ImageWriter(self.publisher_)
        self.start()


    def start(self):
        self.camera.start_recording(self.image_writer, format="mjpeg")



def main():
    print('Hi from rover.')
    rclpy.init()
    cam = RoverCamera()
    rclpy.spin(cam)
    cam.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
