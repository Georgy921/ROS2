import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64
class NumberPublisher(Node):

    def __init__(self):
        super().__init__('number_publisher')
        self.declare_parameter('n', 50)
        self.declare_parameter('publish_rate', 1.0)
        self.n = self.get_parameter('n').get_parameter_value().integer_value
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        if self.n < 1:
            self.n = 50
        self.current_number = 1
        self.publisher_ = self.create_publisher(Int64, 'numbers', 10)
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info(f'NumberPublisher started: 1 to {self.n} at {publish_rate} Hz')

    def timer_callback(self):
        msg = Int64()
        msg.data = self.current_number
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {self.current_number}')
        self.current_number += 1
        if self.current_number > self.n:
            self.current_number = 1
            self.get_logger().info('Restarting from 1')


def main(args=None):
    rclpy.init(args=args)
    node = NumberPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Stopped')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
