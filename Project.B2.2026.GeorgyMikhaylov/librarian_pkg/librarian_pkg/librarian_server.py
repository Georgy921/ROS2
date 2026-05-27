"""
Action Server — управляет черепахой-библиотекарем.
Action: /library/deliver_book

Логика:
1. Едет к книге (x, BOOK_Y)
2. Останавливается на 1 сек ("берёт книгу")
3. Едет на стол выдачи (1, 1)
4. Останавливается ("кладёт книгу")
5. Возвращается на базу (5, 5)
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

from librarian_interfaces.action import DeliverBook
from librarian_interfaces.srv import RequestBook

import math
import time


DESK_X = 1.0
DESK_Y = 1.0
BASE_X = 5.5
BASE_Y = 5.5
SPEED = 2.0
ANGLE_SPEED = 2.0
TOLERANCE = 0.1


class LibrarianServer(Node):

    def __init__(self, book_status: dict):
        super().__init__('librarian_server')

        self.book_status = book_status
        self.current_pose = None

        self._cb_group = ReentrantCallbackGroup()

        # Action сервер
        self._action_server = ActionServer(
            node=self,
            action_type=DeliverBook,
            action_name='/library/deliver_book',
            execute_callback=self.execute_callback,
            callback_group=self._cb_group,
        )

        # Сервис-клиент для поиска книги
        self._book_client = self.create_client(
            RequestBook,
            '/library/request_book',
            callback_group=self._cb_group,
        )

        # Подписка на позицию черепахи
        self.pose_sub = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10,
            callback_group=self._cb_group,
        )

        # Паблишер команд движения
        self.cmd_pub = self.create_publisher(
            Twist, '/turtle1/cmd_vel', 10
        )

        self.get_logger().info('Librarian Action Server ready!')

    def pose_callback(self, msg: Pose):
        self.current_pose = msg

    def send_feedback(self, goal_handle, stage, x, y):
        fb = DeliverBook.Feedback()
        fb.stage = stage
        fb.current_x = x
        fb.current_y = y
        goal_handle.publish_feedback(fb)
        self.get_logger().info(f'[{stage}] pos=({x:.2f},{y:.2f})')

    def move_to(self, goal_handle, target_x, target_y, stage: str):
        """Едет к точке (target_x, target_y). Публикует feedback."""

        rate_hz = 20
        rate_sec = 1.0 / rate_hz

        while rclpy.ok():
            if self.current_pose is None:
                time.sleep(0.1)
                continue

            cx = self.current_pose.x
            cy = self.current_pose.y
            ctheta = self.current_pose.theta

            dx = target_x - cx
            dy = target_y - cy
            dist = math.sqrt(dx ** 2 + dy ** 2)

            if dist < TOLERANCE:
                self.stop()
                return True

            # Угол к цели
            target_angle = math.atan2(dy, dx)
            angle_err = target_angle - ctheta

            # Нормализация угла [-pi, pi]
            while angle_err > math.pi:
                angle_err -= 2 * math.pi
            while angle_err < -math.pi:
                angle_err += 2 * math.pi

            twist = Twist()
            twist.linear.x = min(SPEED * dist, SPEED)
            twist.angular.z = ANGLE_SPEED * angle_err
            self.cmd_pub.publish(twist)

            self.send_feedback(goal_handle, stage, cx, cy)

            time.sleep(rate_sec)

        return False

    def stop(self):
        self.cmd_pub.publish(Twist())

    def execute_callback(self, goal_handle: ServerGoalHandle):
        book_name = goal_handle.request.book_name
        book_x = goal_handle.request.book_x
        book_y = goal_handle.request.book_y

        result = DeliverBook.Result()

        self.get_logger().info(
            f'Начинаю доставку книги "{book_name}" '
            f'с позиции ({book_x}, {book_y})'
        )
        timeout = time.time() + 5.0
        while self.current_pose is None and time.time() < timeout:
            time.sleep(0.1)

        if self.current_pose is None:
            result.success = False
            result.message = 'Не получена позиция черепахи!'
            goal_handle.abort()
            return result

        # 1. Едем к книге
        self.move_to(goal_handle, book_x, book_y, 'Еду к книге')

        # 2. Берём книгу
        self.get_logger().info(f'Беру книгу "{book_name}"...')
        self.book_status[book_name] = False  # книга выдана
        self.send_feedback(
            goal_handle, 'Беру книгу', book_x, book_y
        )
        time.sleep(1.0)

        # 3. Едем на стол выдачи
        self.move_to(goal_handle, DESK_X, DESK_Y, 'Везу на стол выдачи')

        # 4. Кладём книгу
        self.get_logger().info('Кладу книгу на стол выдачи.')
        self.send_feedback(
            goal_handle, 'Кладу на стол', DESK_X, DESK_Y
        )
        time.sleep(1.0)

        # 5. Возвращаемся на базу
        self.move_to(goal_handle, BASE_X, BASE_Y, 'Возвращаюсь на базу')

        self.get_logger().info('Вернулся на базу. Задача выполнена!')

        result.success = True
        result.message = f'Книга "{book_name}" успешно доставлена!'
        goal_handle.succeed()
        return result


# ─── Главный узел-оркестратор ────────────────────────────────────────────────

class LibrarianMain(Node):
    """
    Главный узел — объединяет все три механизма:
    - Topic publisher (статус книг)
    - Service server (поиск книги)
    - Action server (движение черепахи)
    """

    def __init__(self):
        super().__init__('librarian_main')

        # Общий словарь состояния книг
        self.book_status = {
            'Война и мир':        True,
            'Преступление':       True,
            'Мастер и Маргарита': True,
            'Евгений Онегин':     True,
            'Идиот':              True,
            'Анна Каренина':      True,
            'Отцы и дети':        True,
            'Мёртвые души':       True,
        }

        self._cb_group = ReentrantCallbackGroup()

        # Topic: публикуем статус книг
        self.status_pub = self.create_publisher(
            __import__(
                'librarian_interfaces.msg',
                fromlist=['Book']
            ).Book,
            '/library/books_status', 10
        )
        self.timer = self.create_timer(
            1.0, self.publish_status, callback_group=self._cb_group
        )

        # Service: принимаем запрос книги
        from librarian_interfaces.srv import RequestBook
        self.srv = self.create_service(
            RequestBook,
            '/library/request_book',
            self.handle_request,
            callback_group=self._cb_group,
        )

        # Action: управляем черепахой
        from librarian_interfaces.action import DeliverBook
        from geometry_msgs.msg import Twist
        from turtlesim.msg import Pose

        self.current_pose = None
        self.cmd_pub = self.create_publisher(
            Twist, '/turtle1/cmd_vel', 10
        )
        self.pose_sub = self.create_subscription(
            Pose, '/turtle1/pose',
            lambda msg: setattr(self, 'current_pose', msg),
            10, callback_group=self._cb_group,
        )

        self._action_server = ActionServer(
            node=self,
            action_type=DeliverBook,
            action_name='/library/deliver_book',
            execute_callback=self.execute_deliver,
            callback_group=self._cb_group,
        )

        self.get_logger().info('Librarian Main Node is ready!')

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

    def publish_status(self):
            from librarian_interfaces.msg import Book
            for name, x in self.BOOKS.items():
                msg = Book()
                msg.name = name
                msg.x = x
                msg.y = self.BOOK_Y
                msg.is_available = self.book_status.get(name, True)
                self.status_pub.publish(msg)

    def handle_request(self, request, response):
        name = request.book_name
        self.get_logger().info(f'Запрос: "{name}"')

        if name not in self.BOOKS:
            response.success = False
            response.message = f'Книга "{name}" не найдена!'
            response.x = 0.0
            response.y = 0.0
            return response

        if not self.book_status.get(name, True):
            response.success = False
            response.message = f'Книга "{name}" уже выдана!'
            response.x = 0.0
            response.y = 0.0
            return response

        response.success = True
        response.message = f'Найдена! Начинаю доставку.'
        response.x = self.BOOKS[name]
        response.y = self.BOOK_Y
        return response

    def move_to(self, goal_handle, tx, ty, stage):
        from geometry_msgs.msg import Twist
        while rclpy.ok():
            if self.current_pose is None:
                time.sleep(0.05)
                continue
            cx = self.current_pose.x
            cy = self.current_pose.y
            dx, dy = tx - cx, ty - cy
            dist = math.sqrt(dx ** 2 + dy ** 2)
            if dist < TOLERANCE:
                self.cmd_pub.publish(Twist())
                return

            angle = math.atan2(dy, dx)
            err = angle - self.current_pose.theta
            while err > math.pi:
                err -= 2 * math.pi
            while err < -math.pi:
                err += 2 * math.pi

            t = Twist()
            t.linear.x = min(SPEED * dist, SPEED)
            t.angular.z = ANGLE_SPEED * err
            self.cmd_pub.publish(t)

            fb = DeliverBook.Feedback()
            fb.stage = stage
            fb.current_x = cx
            fb.current_y = cy
            goal_handle.publish_feedback(fb)
            time.sleep(0.05)

    def execute_deliver(self, goal_handle: ServerGoalHandle):
        from librarian_interfaces.action import DeliverBook
        book_name = goal_handle.request.book_name
        book_x = goal_handle.request.book_x
        book_y = goal_handle.request.book_y

        self.get_logger().info(f'Доставляю: "{book_name}"')

        # Ждём позицию черепахи
        t = time.time()
        while self.current_pose is None and time.time() - t < 5:
            time.sleep(0.1)

        # 1. К книге
        self.move_to(goal_handle, book_x, book_y, 'Еду к книге')

        # 2. Берём
        self.book_status[book_name] = False
        self.get_logger().info('Беру книгу...')
        time.sleep(1.0)

        # 3. На стол
        self.move_to(goal_handle, DESK_X, DESK_Y, 'Везу на стол')

        # 4. Кладём
        self.get_logger().info('Кладу на стол выдачи.')
        time.sleep(1.0)

        # 5. На базу
        self.move_to(goal_handle, BASE_X, BASE_Y, 'Возвращаюсь')

        result = DeliverBook.Result()
        result.success = True
        result.message = f'"{book_name}" доставлена!'
        goal_handle.succeed()
        self.get_logger().info('Готово!')
        return result


def main(args=None):
    rclpy.init(args=args)
    node = LibrarianMain()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__== '__main__':
    main()