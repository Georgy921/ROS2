#!/usr/bin/env python3

import sys

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from librarian_interfaces.srv import RequestBook
from librarian_interfaces.action import DeliverBook


class LibrarianClient(Node):

    def __init__(self):
        super().__init__('librarian_client')

        self._srv_client = self.create_client(
            RequestBook,
            '/library/request_book'
        )

        self._action_client = ActionClient(
            self,
            DeliverBook,
            '/library/deliver_book'
        )

        self._done = False

    def run(self, book_name: str):
        self.get_logger().info(f'Запрашиваю книгу: "{book_name}"')

        # 1. Сервис
        if not self._srv_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Сервис /library/request_book недоступен!')
            self._done = True
            return

        req = RequestBook.Request()
        req.book_name = book_name

        future = self._srv_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if not future.done():
            self.get_logger().error('Нет ответа от сервиса!')
            self._done = True
            return

        resp = future.result()
        self.get_logger().info(
            f'Ответ сервиса: success={resp.success}, '
            f'message="{resp.message}", x={resp.x:.2f}, y={resp.y:.2f}'
        )

        if not resp.success:
            self.get_logger().warn('Книга недоступна. Action запускаться не будет.')
            self._done = True
            return

        # 2. Action
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action /library/deliver_book недоступен!')
            self._done = True
            return

        goal = DeliverBook.Goal()
        goal.book_name = book_name
        goal.book_x = float(resp.x)
        goal.book_y = float(resp.y)

        send_future = self._action_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback,
        )
        send_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f'[{fb.stage}] ({fb.current_x:.2f}, {fb.current_y:.2f})'
        )

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal отклонён action сервером!')
            self._done = True
            return

        self.get_logger().info('Goal принят. Жду результат...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result_response = future.result()
        status = result_response.status
        result = result_response.result

        self.get_logger().info(
            f'Результат: success={result.success}, '
            f'status={status}, message="{result.message}"'
        )
        self._done = True


def main(args=None):
    rclpy.init(args=args)

    node = LibrarianClient()

    book_name = 'Война и мир'
    if len(sys.argv) > 1:
        book_name = ' '.join(sys.argv[1:])

    node.run(book_name)

    try:
        while rclpy.ok() and not node._done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info('Клиент остановлен пользователем.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()