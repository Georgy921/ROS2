import rclpy
from rclpy.node import Node
from librarian_interfaces.msg import Book


# Список книг: name -> (x, y)
BOOKS = {
    'Война и мир':      2.0,
    'Преступление':     3.0,
    'Мастер и Маргарита': 4.0,
    'Евгений Онегин':   5.0,
    'Идиот':            6.0,
    'Анна Каренина':    7.0,
    'Отцы и дети':      8.0,
    'Мёртвые души':     9.0,
}
BOOK_Y = 9.0


class StatusPublisher(Node):

    def __init__(self, book_status: dict):
        super().__init__('status_publisher')

        self.book_status = book_status  # shared dict с состоянием книг

        self.publisher = self.create_publisher(
            Book, '/library/books_status', 10
        )

        self.timer = self.create_timer(1.0, self.publish_status)
        self.get_logger().info('Status Publisher started.')

    def publish_status(self):
        for name, x in BOOKS.items():
            msg = Book()
            msg.name = name
            msg.x = x
            msg.y = BOOK_Y
            msg.is_available = self.book_status.get(name, True)
            self.publisher.publish(msg)