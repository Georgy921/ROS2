import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

from librarian_interfaces.msg import Book
from librarian_interfaces.srv import RequestBook
from librarian_interfaces.action import DeliverBook


BOOKS = {
    'Война и мир': 2.0,
    'Преступление': 3.0,
    'Мастер и Маргарита': 4.0,
    'Евгений Онегин': 5.0,
    'Идиот': 6.0,
    'Анна Каренина': 7.0,
    'Отцы и дети': 8.0,
    'Мёртвые души': 9.0,
}

BOOK_Y = 9.0

DESK_X = 1.0
DESK_Y = 1.0

BASE_X = 5.5
BASE_Y = 5.5

TOLERANCE = 0.20


class LibrarianServer(Node):

    def __init__(self):
        super().__init__('librarian_server')

        self._cb_group = ReentrantCallbackGroup()
        self.current_pose = None

        # True = доступна, False = уже выдана
        self.book_status = {name: True for name in BOOKS}

        # Topic publisher
        self.status_pub = self.create_publisher(
            Book,
            '/library/books_status',
            10
        )
        self.status_timer = self.create_timer(1.0, self.publish_status)

        # Service server
        self.request_service = self.create_service(
            RequestBook,
            '/library/request_book',
            self.handle_request,
            callback_group=self._cb_group,
        )

        # Pose subscriber
        self.pose_sub = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10,
            callback_group=self._cb_group,
        )

        # Velocity publisher
        self.cmd_pub = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            10
        )

        # ВАЖНО: action server сохраняем в self
        self._action_server = ActionServer(
            node=self,
            action_type=DeliverBook,
            action_name='/library/deliver_book',
            execute_callback=self.execute_deliver,
            callback_group=self._cb_group,
        )

        self.get_logger().info('Librarian Server ready!')

    def pose_callback(self, msg: Pose):
        self.current_pose = msg

    def publish_status(self):
        for name, x in BOOKS.items():
            msg = Book()
            msg.name = name
            msg.x = float(x)
            msg.y = float(BOOK_Y)
            msg.is_available = self.book_status.get(name, True)
            self.status_pub.publish(msg)

    def handle_request(self, request, response):
        book_name = request.book_name.strip()
        self.get_logger().info(f'Запрос книги: "{book_name}"')

        if book_name not in BOOKS:
            response.success = False
            response.message = f'Книга "{book_name}" не найдена!'
            response.x = 0.0
            response.y = 0.0
            return response

        if not self.book_status.get(book_name, True):
            response.success = False
            response.message = f'Книга "{book_name}" уже выдана!'
            response.x = 0.0
            response.y = 0.0
            return response

        response.success = True
        response.message = f'Книга "{book_name}" найдена. Начинаю доставку.'
        response.x = float(BOOKS[book_name])
        response.y = float(BOOK_Y)
        return response

    def stop(self):
        self.cmd_pub.publish(Twist())

    def clamp(self, value, min_value, max_value):
        return max(min(value, max_value), min_value)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def publish_feedback(self, goal_handle, stage, x, y):
        feedback = DeliverBook.Feedback()
        feedback.stage = stage
        feedback.current_x = float(x)
        feedback.current_y = float(y)
        goal_handle.publish_feedback(feedback)

    def move_to(self, goal_handle, tx, ty, stage, timeout=30.0):
        """
        Улучшенное движение:
        1) сначала поворот к цели
        2) потом движение вперёд
        3) если угол снова сильно ушёл — возвращаемся к повороту

        Возвращает:
          'reached'  - цель достигнута
          'canceled' - goal отменён
          'timeout'  - превышен timeout
          'shutdown' - rclpy завершён
        """
        start_time = time.time()
        phase = 'turn'

        self.get_logger().info(
            f'[{stage}] Старт движения к ({tx:.2f}, {ty:.2f})'
        )

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.stop()
                self.get_logger().warn(f'[{stage}] Goal отменён')
                return 'canceled'

            if self.current_pose is None:
                time.sleep(0.03)
                continue

            if time.time() - start_time > timeout:
                self.stop()
                self.get_logger().error(
                    f'[{stage}] Таймаут движения к ({tx:.2f}, {ty:.2f})'
                )
                return 'timeout'

            cx = self.current_pose.x
            cy = self.current_pose.y
            ctheta = self.current_pose.theta

            dx = tx - cx
            dy = ty - cy
            dist = math.sqrt(dx * dx + dy * dy)

            target_angle = math.atan2(dy, dx)
            angle_error = self.normalize_angle(target_angle - ctheta)

            self.publish_feedback(goal_handle, stage, cx, cy)

            if dist < TOLERANCE:
                self.stop()
                self.get_logger().info(
                    f'[{stage}] Цель достигнута: ({cx:.2f}, {cy:.2f})'
                )
                return 'reached'

            twist = Twist()

            # ───── ФАЗА 1: сначала только поворот ─────
            if phase == 'turn':
                # Если уже смотрим почти на цель — переходим к движению
                if abs(angle_error) < 0.05:
                    self.stop()
                    phase = 'drive'
                    time.sleep(0.05)
                    continue

                # Адаптивная скорость поворота
                if abs(angle_error) > 1.0:
                    angular_speed = 2.5
                elif abs(angle_error) > 0.30:
                    angular_speed = 1.2
                else:
                    angular_speed = 0.4

                twist.linear.x = 0.0
                twist.angular.z = angular_speed if angle_error > 0.0 else -angular_speed

            # ───── ФАЗА 2: движение вперёд ─────
            else:
                # Если сильно сбились по углу — снова поворот
                if abs(angle_error) > 0.60:
                    self.stop()
                    phase = 'turn'
                    time.sleep(0.05)
                    continue

                # Линейная скорость зависит от расстояния
                if dist > 2.0:
                    linear_speed = 1.8
                elif dist > 0.7:
                    linear_speed = min(1.2 * dist, 1.2)
                else:
                    linear_speed = max(0.10, 0.6 * dist)

                # Если угол неидеальный — немного замедляемся
                if abs(angle_error) > 0.25:
                    linear_speed *= 0.6

                # Плавная коррекция курса
                angular_correction = self.clamp(2.0 * angle_error, -1.0, 1.0)

                twist.linear.x = linear_speed
                twist.angular.z = angular_correction

            self.cmd_pub.publish(twist)
            time.sleep(0.03)

        self.stop()
        return 'shutdown'

    def execute_deliver(self, goal_handle: ServerGoalHandle):
        result = DeliverBook.Result()


        try:
            book_name = goal_handle.request.book_name
            book_x = goal_handle.request.book_x
            book_y = goal_handle.request.book_y

            self.get_logger().info(
                f'Получен goal: "{book_name}" ({book_x:.2f}, {book_y:.2f})'
            )

            # Проверка книги
            if book_name not in BOOKS:
                result.success = False
                result.message = f'Книга "{book_name}" не найдена!'
                self.get_logger().warn(result.message)
                goal_handle.abort()
                return result

            if not self.book_status.get(book_name, True):
                result.success = False
                result.message = f'Книга "{book_name}" уже выдана!'
                self.get_logger().warn(result.message)
                goal_handle.abort()
                return result

            # Ждём pose от turtlesim
            wait_start = time.time()
            while self.current_pose is None and time.time() - wait_start < 5.0:
                time.sleep(0.1)

            if self.current_pose is None:
                result.success = False
                result.message = 'Не получена позиция черепахи /turtle1/pose'
                self.get_logger().error(result.message)
                goal_handle.abort()
                return result

            # 1. Едем к книге
            move_status = self.move_to(
                goal_handle,
                book_x,
                book_y,
                'Еду к книге',
                timeout=35.0
            )

            if move_status == 'canceled':
                result.success = False
                result.message = 'Goal отменён при движении к книге'
                self.get_logger().warn(result.message)
                goal_handle.canceled()
                return result

            if move_status != 'reached':
                result.success = False
                result.message = f'Не удалось доехать до книги "{book_name}"'
                self.get_logger().error(result.message)
                goal_handle.abort()
                return result

            # 2. Берём книгу
            self.book_status[book_name] = False
            self.publish_feedback(goal_handle, 'Беру книгу', book_x, book_y)
            self.get_logger().info(f'Беру книгу "{book_name}"...')
            time.sleep(1.0)

            # 3. Везём на стол
            move_status = self.move_to(
                goal_handle,
                DESK_X,
                DESK_Y,
                'Везу на стол',
                timeout=35.0
            )

            if move_status == 'canceled':
                result.success = False
                result.message = 'Goal отменён по пути к столу выдачи'
                self.get_logger().warn(result.message)
                goal_handle.canceled()
                return result

            if move_status != 'reached':
                result.success = False
                result.message = f'Не удалось довезти "{book_name}" до стола'
                self.get_logger().error(result.message)
                goal_handle.abort()
                return result

            # 4. Кладём на стол
            self.publish_feedback(goal_handle, 'Кладу на стол', DESK_X, DESK_Y)
            self.get_logger().info(f'Кладу "{book_name}" на стол...')
            time.sleep(1.0)

            # 5. Возвращаемся на базу
            move_status = self.move_to(
                goal_handle,
                BASE_X,
                BASE_Y,
                'Возвращаюсь',
                timeout=35.0
            )

            if move_status == 'canceled':
                result.success = False
                result.message = 'Goal отменён при возврате на базу'
                self.get_logger().warn(result.message)
                goal_handle.canceled()
                return result

            if move_status != 'reached':
                result.success = False
                result.message = (
                    f'Книга "{book_name}" доставлена, '
                    f'но не удалось вернуться на базу'
                )
                self.get_logger().error(result.message)
                goal_handle.abort()
                return result

            # УСПЕШНЫЙ РЕЗУЛЬТАТ
            result.success = True
            result.message = f'Книга "{book_name}" успешно доставлена!'

            self.get_logger().info(
                f'Перед return result: success={result.success}, '
                f'message="{result.message}"'
            )

            goal_handle.succeed()
            return result

        except Exception as e:
            self.stop()
            result.success = False
            result.message = f'Внутренняя ошибка action сервера: {e}'
            self.get_logger().error(result.message)

            if goal_handle.is_active:
                goal_handle.abort()

            return result


def main(args=None):
    rclpy.init(args=args)

    node = LibrarianServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Остановка librarian_server...')
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()