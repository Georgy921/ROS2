import sys
import rclpy
from rclpy.node import Node
from name_finder_interfaces.srv import FindNames


class NameFinderClient(Node):

    def __init__(self):
        super().__init__('name_finder_client')
        self.client = self.create_client(FindNames, 'find_names')

        self.get_logger().info('Waiting for server...')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Server not available, waiting...')

        self.get_logger().info('Server is ready.')

    def send_request(self, text):
        request = FindNames.Request()
        request.text = text

        self.get_logger().info(f'Sending: "{text}"')

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        return future.result()


def main(args=None):
    rclpy.init(args=args)
    node = NameFinderClient()

    if len(sys.argv) > 1:
        text = ' '.join(sys.argv[1:])
    else:
        while True:
            text = input("Please write some text or /break to stop client: ")
            if text == "/break":
                break
            response = node.send_request(text)
            if response is not None:
                print(f'Input : {text}')
                print(f'Result: {list(response.names)}')
                print('--')
            else:
                node.get_logger().error('Service call failed.')

    node.destroy_node()
    rclpy.shutdown()

        

    


if __name__ == '__main__':
    main()