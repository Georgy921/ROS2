import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64
class PrimeFactorSubscriber(Node):

    def __init__(self):
        super().__init__('prime_factor_subscriber')
        self.subscription = self.create_subscription(
            Int64,
            'numbers',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(Int64, 'largest_prime_factor', 10)
        self.get_logger().info('PrimeFactorSubscriber started')

    def largest_prime_factor(self, n):
        if n <= 1:
            return n
        largest_factor = 1
        number = n
        while number % 2 == 0:
            largest_factor = 2
            number //= 2
        divisor = 3
        while divisor * divisor <= number:
            while number % divisor == 0:
                largest_factor = divisor
                number //= divisor
            divisor += 2
        if number > 1:
            largest_factor = number
        return largest_factor

    def listener_callback(self, msg):
        received = msg.data
        prime = self.largest_prime_factor(received)
        self.get_logger().info(f'Received: {received} -> Largest prime factor: {prime}')
        result_msg = Int64()
        result_msg.data = prime
        self.publisher_.publish(result_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PrimeFactorSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Stopped')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
