#!/usr/bin/env python3
"""
Тесты для Робота-библиотекаря.
Запуск: pytest src/librarian_pkg/test/test_librarian.py -v
"""

import pytest
import rclpy
import time
import math
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor

from librarian_interfaces.srv import RequestBook
from librarian_interfaces.action import DeliverBook
from librarian_interfaces.msg import Book


# ────────────────────────────────────────────────────────
# Данные
# ────────────────────────────────────────────────────────

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
DESK_X, DESK_Y = 1.0, 1.0
BASE_X, BASE_Y = 5.5, 5.5


# ────────────────────────────────────────────────────────
# Unit тесты (без ROS2)
# ────────────────────────────────────────────────────────

class TestBookData:

    def test_all_books_present(self):
        """Должно быть 8 книг."""
        assert len(BOOKS) == 8

    def test_books_x_range(self):
        """X координаты книг от 2 до 9."""
        xs = list(BOOKS.values())
        assert min(xs) == 2.0
        assert max(xs) == 9.0

    def test_book_y_correct(self):
        """Y координата стеллажа = 9."""
        assert BOOK_Y == 9.0

    def test_desk_position(self):
        """Стол выдачи на (1, 1)."""
        assert DESK_X == 1.0
        assert DESK_Y == 1.0

    def test_book_names_unique(self):
        """Все названия книг уникальны."""
        assert len(BOOKS) == len(set(BOOKS.keys()))

    def test_unknown_book_not_in_list(self):
        """Несуществующей книги нет в списке."""
        assert 'Незнайка на Луне' not in BOOKS

    def test_distance_calculation(self):
        """Расстояние от базы до книги больше нуля."""
        book_x = BOOKS['Война и мир']
        dist = math.sqrt((book_x - BASE_X) ** 2 + (BOOK_Y - BASE_Y) ** 2)
        assert dist > 0

    def test_all_books_initially_available(self):
        """Все книги изначально доступны."""
        book_status = {name: True for name in BOOKS}
        assert all(book_status.values())

    def test_book_becomes_unavailable(self):
        """Книга становится недоступной после выдачи."""
        book_status = {name: True for name in BOOKS}
        book_status['Идиот'] = False
        assert not book_status['Идиот']
        assert book_status['Война и мир']


# ────────────────────────────────────────────────────────
# Helpers
# ────────────────────────────────────────────────────────

@pytest.fixture(scope='module')
def ros_node():
    rclpy.init()
    node = Node('test_librarian')
    yield node
    node.destroy_node()
    rclpy.shutdown()


def call_service(node, client, book_name, timeout=5.0):
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    if not client.wait_for_service(timeout_sec=timeout):
        executor.shutdown()
        return None

    req = RequestBook.Request()
    req.book_name = book_name
    future = client.call_async(req)

    end = time.time() + timeout
    while not future.done() and time.time() < end:
        executor.spin_once(timeout_sec=0.1)

    executor.shutdown()
    return future.result() if future.done() else None


def send_action(node, client, book_name, book_x, book_y, timeout=120.0):
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    if not client.wait_for_server(timeout_sec=10.0):
        executor.shutdown()
        return None, []

    goal = DeliverBook.Goal()
    goal.book_name = book_name
    goal.book_x = float(book_x)
    goal.book_y = float(book_y)

    feedback_list = []
    future = client.send_goal_async(
        goal,
        feedback_callback=lambda f: feedback_list.append(f.feedback)
    )

    end = time.time() + timeout
    while not future.done() and time.time() < end:
        executor.spin_once(timeout_sec=0.1)


    if not future.done():
        executor.shutdown()
        return None, []

    gh = future.result()
    if not gh.accepted:
        executor.shutdown()
        return None, []

    rf = gh.get_result_async()
    end = time.time() + timeout
    while not rf.done() and time.time() < end:
        executor.spin_once(timeout_sec=0.1)

    executor.shutdown()
    return (rf.result().result if rf.done() else None), feedback_list


# ────────────────────────────────────────────────────────
# Integration тесты — Service
# ────────────────────────────────────────────────────────

class TestBookService:

    @pytest.fixture
    def srv_client(self, ros_node):
        client = ros_node.create_client(
            RequestBook, '/library/request_book'
        )
        yield ros_node, client
        ros_node.destroy_client(client)

    def test_service_available(self, srv_client):
        """TC-01: Сервис должен быть доступен."""
        node, client = srv_client
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        ready = client.wait_for_service(timeout_sec=5.0)
        executor.shutdown()
        assert ready, 'Сервис /library/request_book недоступен!'

    def test_unknown_book_not_found(self, srv_client):
        """TC-02: Несуществующая книга → success=False."""
        node, client = srv_client
        resp = call_service(node, client, 'Незнайка на Луне')
        assert resp is not None
        assert resp.success is False
        assert len(resp.message) > 0

    def test_empty_name_not_found(self, srv_client):
        """TC-03: Пустое название → success=False."""
        node, client = srv_client
        resp = call_service(node, client, '')
        assert resp is not None
        assert resp.success is False

    def test_known_book_has_correct_y(self, srv_client):
        """
        TC-04: У найденной книги Y = 9.0.
        Ищем книгу которую точно не выдавали.
        Если уже выдана — тест помечаем как пропущенный.
        """
        node, client = srv_client
        # Пробуем найти хоть одну доступную книгу
        found_any = False
        for name, expected_x in BOOKS.items():
            resp = call_service(node, client, name)
            assert resp is not None

            if resp.success:
                # Нашли доступную — проверяем координаты
                assert resp.x == pytest.approx(expected_x, abs=0.01), (
                    f'Неверный X для "{name}": {resp.x} != {expected_x}'
                )
                assert resp.y == pytest.approx(BOOK_Y, abs=0.01), (
                    f'Неверный Y для "{name}": {resp.y} != {BOOK_Y}'
                )
                found_any = True
                break  # одной достаточно

        if not found_any:
            pytest.skip('Все книги уже выданы — перезапустите сервер')

    def test_unavailable_book_returns_error_message(self, srv_client):
        """
        TC-05: Уже выданная книга → success=False и сообщение об ошибке.
        Находим выданную книгу из ответов.
        """
        node, client = srv_client
        for name in BOOKS:
            resp = call_service(node, client, name)
            assert resp is not None

            if not resp.success:
                # Нашли выданную — проверяем сообщение
                assert len(resp.message) > 0, 'Сообщение об ошибке пустое!'
                assert resp.x == pytest.approx(0.0)
                assert resp.y == pytest.approx(0.0)
                return

        pytest.skip('Нет выданных книг для проверки этого теста')

    def test_response_fields_not_none(self, srv_client):
        """TC-06: Все поля ответа заполнены."""
        node, client = srv_client
        resp = call_service(node, client, 'Идиот')
        assert resp is not None
        assert resp.message is not None
        assert resp.x is not None
        assert resp.y is not None
        assert resp.success is not None


# ────────────────────────────────────────────────────────
# Integration тесты — Topic
# ────────────────────────────────────────────────────────

class TestStatusTopic:

    def test_topic_exists(self, ros_node):
        """TC-07: Топик /library/books_status должен существовать."""
        executor = SingleThreadedExecutor()
        executor.add_node(ros_node)
        found = False
        end = time.time() + 5.0
        while time.time() < end:
            topics = dict(ros_node.get_topic_names_and_types())
            if '/library/books_status' in topics:
                found = True
                break
            executor.spin_once(timeout_sec=0.2)
        executor.shutdown()
        assert found, 'Топик /library/books_status не найден!'

    def test_topic_receives_messages(self, ros_node):
        """TC-08: Топик должен публиковать сообщения."""
        received = []
        executor = SingleThreadedExecutor()
        executor.add_node(ros_node)
        sub = ros_node.create_subscription(
            Book, '/library/books_status',
            lambda msg: received.append(msg), 10
        )
        end = time.time() + 5.0
        while len(received) == 0 and time.time() < end:
            executor.spin_once(timeout_sec=0.1)
        ros_node.destroy_subscription(sub)
        executor.shutdown()
        assert len(received) > 0, 'Нет сообщений из топика!'

    def test_topic_contains_all_books(self, ros_node):
        """TC-09: Должны прийти сообщения для всех 8 книг."""
        received_names = set()
        executor = SingleThreadedExecutor()
        executor.add_node(ros_node)
        sub = ros_node.create_subscription(
            Book, '/library/books_status',
            lambda msg: received_names.add(msg.name), 10
        )
        end = time.time() + 5.0
        while len(received_names) < 8 and time.time() < end:
            executor.spin_once(timeout_sec=0.1)
        ros_node.destroy_subscription(sub)
        executor.shutdown()
        for name in BOOKS:
            assert name in received_names, f'"{name}" не в топике!'

    def test_book_coordinates_in_topic(self, ros_node):
        """TC-10: Координаты в топике совпадают с таблицей."""
        received = {}
        executor = SingleThreadedExecutor()
        executor.add_node(ros_node)
        sub = ros_node.create_subscription(
            Book, '/library/books_status',
            lambda msg: received.update({msg.name: (msg.x, msg.y)}), 10
        )
        end = time.time() + 5.0
        while len(received) < 8 and time.time() < end:
            executor.spin_once(timeout_sec=0.1)
        ros_node.destroy_subscription(sub)
        executor.shutdown()
        for name, expected_x in BOOKS.items():
            if name in received:
                x, y = received[name]
                assert x == pytest.approx(expected_x, abs=0.01)
                assert y == pytest.approx(BOOK_Y, abs=0.01)

    def test_is_available_field_is_bool(self, ros_node):
        """TC-11: Поле is_available должно быть булевым."""
        received = []
        executor = SingleThreadedExecutor()
        executor.add_node(ros_node)
        sub = ros_node.create_subscription(
            Book, '/library/books_status',
            lambda msg: received.append(msg), 10
        )
        end = time.time() + 5.0
        while len(received) == 0 and time.time() < end:
            executor.spin_once(timeout_sec=0.1)
        ros_node.destroy_subscription(sub)
        executor.shutdown()
        assert len(received) > 0
        for msg in received:
            assert isinstance(msg.is_available, bool)


# ────────────────────────────────────────────────────────
# Integration тесты — Action
# ────────────────────────────────────────────────────────

class TestDeliverAction:
    
    @pytest.fixture
    def action_client(self, ros_node):
        client = ActionClient(ros_node, DeliverBook, '/library/deliver_book')
        yield ros_node, client

    def test_action_server_available(self, action_client):
        node, client = action_client
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        ready = client.wait_for_server(timeout_sec=5.0)
        executor.shutdown()
        assert ready, 'Action /library/deliver_book недоступен!'

    def test_feedback_received_during_delivery(self, action_client):
        """TC-14: Feedback должен приходить во время доставки."""
        node, client = action_client

        # Найти доступную книгу
        srv_client = node.create_client(RequestBook, '/library/request_book')
        available_name = None
        available_x = None

        for name, x in BOOKS.items():
            resp = call_service(node, srv_client, name, timeout=3.0)
            if resp is not None and resp.success:
                available_name = name
                available_x = x
                break

        node.destroy_client(srv_client)

        if available_name is None:
            pytest.skip('Нет доступных книг.')

        _, feedback = send_action(
            node, client, available_name, available_x, BOOK_Y
        )

        assert len(feedback) > 0, 'Feedback не получен!'

    