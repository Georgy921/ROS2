import rclpy
from rclpy.node import Node
from name_finder_interfaces.srv import FindNames


class NameFinderServer(Node):

    def __init__(self):
        super().__init__('name_finder_server')
        self.srv = self.create_service(
            FindNames,
            'find_names',
            self.handle_request
        )
        self.get_logger().info('Server is ready.')

    def find_names(self, text):
        words = text.split()
        names = []

        for word in words:
            clean_word = word.strip('.,!?;:"()[]{}')
            if len(clean_word) > 0 and clean_word[0].isupper():
                names.append(clean_word)

        return names

    def handle_request(self, request, response):
        text = request.text
        self.get_logger().info(f'Received: "{text}"')

        response.names = self.find_names(text)

        self.get_logger().info(f'Found: {response.names}')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = NameFinderServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()