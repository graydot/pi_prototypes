import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class TestListener(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.get_logger().info('Started')
        self.subscription = self.create_subscription(
            String,
            'raw_images',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


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