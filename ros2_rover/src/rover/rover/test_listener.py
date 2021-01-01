import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np


class TestListener(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.get_logger().info('Started')
        self.subscription = self.create_subscription(
            CompressedImage,
            'raw_images',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.i = 0

    def listener_callback(self, msg):
        # self.get_logger().info('I heard: "%s"' % msg.data)
        nparr = np.frombuffer(msg.data, np.uint8)
        img=cv2.imdecode(nparr, flags=1)
        img_name = './image_%d.jpg' % self.i
        print('Writing image %s' % img_name)
        cv2.imwrite(img_name, img)
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