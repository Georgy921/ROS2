#!/usr/bin/env python3
"""
Client Node — запрашивает книгу через сервис,
затем запускает action доставки.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor

from librarian_interfaces.srv import RequestBook
from librarian_interfaces.action import DeliverBook

import sys
import time


class LibrarianClient(Node):

    def __init__(self):
        super().__init__('librarian_client')

        self._srv_client = self.create_client(
            RequestBook, '/library/request_book'
        )
        self._action_client = ActionClient(
            self, DeliverBook, '/library/deliver_book'
        )

        self._done = False

    def run(self, book_name: str):
        # 1. Вызов сервиса
        self.get_logger().info(f'Запрашиваю книгу: "{book_name}"')

        if not self._srv_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Сервис недоступен!')
            return

        req = RequestBook.Request()
        req.book_name = book_name

        future = self._srv_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if not future.done():
            self.get_logger().error('Нет ответа от сервиса!')
            return

        resp = future.result()
        self.get_logger().info(f'Ответ сервиса: {resp.message}')

        if not resp.success:
            self.get_logger().warn('Книга недоступна. Выход.')
            self._done = True
            return

        # 2. Запуск action
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action сервер недоступен!')
            return

        goal = DeliverBook.Goal()
        goal.book_name = book_name
        goal.book_x = resp.x
        goal.book_y = resp.y

        send_future = self._action_client.send_goal_async(
            goal, feedback_callback=self.feedback_cb
        )
        send_future.add_done_callback(self.goal_response_cb)

    def feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f'[{fb.stage}] ({fb.current_x:.2f}, {fb.current_y:.2f})'
        )

    def goal_response_cb(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().error('Goal отклонён!')
            self._done = True
            return
        gh.get_result_async().add_done_callback(self.result_cb)

    def result_cb(self, future):
        result = future.result().result
        self.get_logger().info(f'Результат: {result.message}')
        self._done = True


def main(args=None):
    rclpy.init(args=args)
    node = LibrarianClient()

    book_name = 'Война и мир'
    if len(sys.argv) > 1:
        book_name = ' '.join(sys.argv[1:])

    node.run(book_name)

    while rclpy.ok() and not node._done:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()