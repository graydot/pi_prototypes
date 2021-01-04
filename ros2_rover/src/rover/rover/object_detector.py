import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage
from PIL import Image
import numpy as np
from io import BytesIO


class ObjectDetector(Node):

    def __init__(self):
        super().__init__('object_detector')
        self.get_logger().info('Started')
        self.subscription = self.create_subscription(
            CompressedImage,
            'raw_images',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        im = Image.open(BytesIO(msg.data))
        img_name = './image_%d.jpg' % self.i
        print('Writing image %s' % img_name)
        im.save(img_name)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    test_listener = TestListener()

    rclpy.spin(test_listener)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    test_listener.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()