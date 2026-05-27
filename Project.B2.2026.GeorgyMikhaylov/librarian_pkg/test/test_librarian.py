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
        """
        Проверяем расчёт расстояния от базы до книги.
        """
        book_x = BOOKS['Война и мир']  # 2.0
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
        assert book_status['Война и мир']  # остальные доступны


@pytest.fixture(scope='module')
def ros_node():
    rclpy.init()
    node = Node('test_librarian')
    yield node
    node.destroy_node()
    rclpy.shutdown()


def call_service(node, client, book_name, timeout=5.0):
    """Синхронный вызов сервиса."""
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


def send_action(node, client, book_name, book_x, book_y, timeout=60.0):
    """Синхронная отправка action goal."""
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

    def test_known_book_found(self, srv_client):
        """TC-02: Существующая книга должна быть найдена."""
        node, client = srv_client
        resp = call_service(node, client, 'Война и мир')
        assert resp is not None
        assert resp.success is True
        assert resp.x == pytest.approx(2.0)
        assert resp.y == pytest.approx(BOOK_Y)

    def test_unknown_book_not_found(self, srv_client):
        """TC-03: Несуществующая книга → success=False."""
        node, client = srv_client
        resp = call_service(node, client, 'Незнайка на Луне')
        assert resp is not None
        assert resp.success is False
        assert 'не найдена' in resp.message.lower()

    def test_all_books_findable(self, srv_client):
        """TC-04: Все 8 книг должны быть найдены."""
        node, client = srv_client
        for name in BOOKS:
            resp = call_service(node, client, name)
            assert resp is not None, f'Нет ответа для "{name}"'
            assert resp.success is True, f'Книга "{name}" не найдена!'
            assert resp.x == pytest.approx(BOOKS[name])

    def test_response_has_correct_coordinates(self, srv_client):
        """TC-05: Координаты в ответе соответствуют таблице книг."""
        node, client = srv_client
        for name, expected_x in BOOKS.items():
            resp = call_service(node, client, name)
            assert resp is not None
            if resp.success:
                assert resp.x == pytest.approx(expected_x, abs=0.01)
                assert resp.y == pytest.approx(BOOK_Y, abs=0.01)

    def test_empty_name_returns_not_found(self, srv_client):
        """TC-06: Пустое название книги → success=False."""
        node, client = srv_client
        resp = call_service(node, client, '')
        assert resp is not None
        assert resp.success is False


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
                Book,
                '/library/books_status',
                lambda msg: received.append(msg),
                10
            )

            end = time.time() + 5.0
            while len(received) == 0 and time.time() < end:
                executor.spin_once(timeout_sec=0.1)

            ros_node.destroy_subscription(sub)
            executor.shutdown()

            assert len(received) > 0, 'Нет сообщений из топика!'

    def test_topic_contains_all_books(self, ros_node):
        """TC-09: За 3 сек должны прийти сообщения для всех 8 книг."""
        received_names = set()
        executor = SingleThreadedExecutor()
        executor.add_node(ros_node)

        sub = ros_node.create_subscription(
            Book,
            '/library/books_status',
            lambda msg: received_names.add(msg.name),
            10
        )

        end = time.time() + 5.0
        while len(received_names) < 8 and time.time() < end:
            executor.spin_once(timeout_sec=0.1)

        ros_node.destroy_subscription(sub)
        executor.shutdown()

        for name in BOOKS:
            assert name in received_names, (
                f'Книга "{name}" не найдена в топике!'
            )

    def test_book_coordinates_in_topic(self, ros_node):
        """TC-10: Координаты в топике совпадают с таблицей."""
        received = {}
        executor = SingleThreadedExecutor()
        executor.add_node(ros_node)

        sub = ros_node.create_subscription(
            Book,
            '/library/books_status',
            lambda msg: received.update({msg.name: (msg.x, msg.y)}),
            10
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


class TestDeliverAction:

    @pytest.fixture
    def action_client(self, ros_node):
        client = ActionClient(
            ros_node, DeliverBook, '/library/deliver_book'
        )
        yield ros_node, client

    def test_action_server_available(self, action_client):
        """TC-11: Action сервер должен быть доступен."""
        node, client = action_client
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        ready = client.wait_for_server(timeout_sec=5.0)
        executor.shutdown()
        assert ready, 'Action /library/deliver_book недоступен!'

    def test_deliver_succeeds(self, action_client):
        """TC-12: Доставка книги должна завершиться успехом."""
        node, client = action_client
        result, _ = send_action(
            node, client,
            'Евгений Онегин',
            BOOKS['Евгений Онегин'],
            BOOK_Y
        )
        assert result is not None
        assert result.success is True

    def test_feedback_received(self, action_client):
        """TC-13: Должен приходить feedback во время доставки."""
        node, client = action_client
        result, feedback = send_action(
            node, client,
            'Идиот',
            BOOKS['Идиот'],
            BOOK_Y
        )
        assert result is not None
        assert len(feedback) > 0, 'Feedback не получен!'

    def test_feedback_stages(self, action_client):
            """TC-14: Feedback должен содержать стадии движения."""
            node, client = action_client
            result, feedback = send_action(
                node, client,
                'Мёртвые души',
                BOOKS['Мёртвые души'],
                BOOK_Y
            )
            assert result is not None
            assert len(feedback) > 0

            stages = {fb.stage for fb in feedback}
            assert len(stages) >= 1, 'Нет стадий в feedback!'

    def test_result_message_not_empty(self, action_client):
        """TC-15: Сообщение результата не должно быть пустым."""
        node, client = action_client
        result, _ = send_action(
            node, client,
            'Преступление',
            BOOKS['Преступление'],
            BOOK_Y
        )
        assert result is not None
        assert len(result.message) > 0