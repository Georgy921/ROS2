import rclpy
from rclpy.node import Node
from librarian_interfaces.srv import RequestBook


BOOKS = {
    'Война и мир':        2.0,
    'Преступление':       3.0,
    'Мастер и Маргарита': 4.0,
    'Евгений Онегин':     5.0,
    'Идиот':              6.0,
    'Анна Каренина':      7.0,
    'Отцы и дети':        8.0,
    'Мёртвые души':       9.0,
}
BOOK_Y = 9.0


class BookService(Node):

    def __init__(self, book_status: dict):
        super().__init__('book_service')

        self.book_status = book_status  # shared dict

        self.srv = self.create_service(
            RequestBook,
            '/library/request_book',
            self.handle_request
        )

        self.get_logger().info('Book Service ready.')

    def handle_request(self, request, response):
        name = request.book_name
        self.get_logger().info(f'Запрос книги: "{name}"')

        if name not in BOOKS:
            response.success = False
            response.message = f'Книга "{name}" не найдена в библиотеке.'
            response.x = 0.0
            response.y = 0.0
            self.get_logger().warn(response.message)
            return response

        if not self.book_status.get(name, True):
            response.success = False
            response.message = f'Книга "{name}" уже выдана!'
            response.x = 0.0
            response.y = 0.0
            self.get_logger().warn(response.message)
            return response

        # Книга найдена и доступна
        response.success = True
        response.message = f'Книга "{name}" найдена. Начинаю доставку.'
        response.x = BOOKS[name]
        response.y = BOOK_Y

        self.get_logger().info(response.message)
        return response