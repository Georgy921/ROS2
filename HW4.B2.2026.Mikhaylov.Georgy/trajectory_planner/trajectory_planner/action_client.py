import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.task import Future

import sys
import math

from trajectory_interfaces.action import Trajectory


class TrajectoryActionClient(Node):
    def __init__(self):
        super().__init__('trajectory_action_client')

        # Create action client
        self._action_client = ActionClient(
            node=self,
            action_type=Trajectory,
            action_name='trajectory_planner',
        )

        self._goal_handle = None
        self._result_future = None
        self._feedback_count = 0
        self._done = False 

        self.get_logger().info('Trajectory Action Client initialized.')

    def send_goal(
        self,
        linear_velocity: float,
        target_x: float,
        target_y: float,
    ) -> None:

        self.get_logger().info(
            f'Waiting for action server...\n'

        )

        # Wait for server to be available (timeout 10s)
        server_ready = self._action_client.wait_for_server(timeout_sec=10.0)

        if not server_ready:
            self.get_logger().error(
                'Action server not available after 10 seconds!'
            )
            return

        self.get_logger().info('Action server is ready!')
        goal_msg = Trajectory.Goal()
        goal_msg.linear_velocity = linear_velocity
        goal_msg.target_x = target_x
        goal_msg.target_y = target_y

        distance = math.sqrt(target_x ** 2 + target_y ** 2)
        expected_steps = distance / linear_velocity

        self.get_logger().info(
            f'\n{"=" * 50}\n'
            f'Sending goal:\n'
            f'  Linear velocity: {linear_velocity:.3f} m/s\n'
            f'  Target point:    ({target_x:.3f}, {target_y:.3f})\n'
            f'  Total distance:  {distance:.3f} m\n'
            f'  Expected steps:  ~{expected_steps:.1f}\n'
            f'{"=" * 50}'
        )

        self._feedback_count = 0

        # Send goal asynchronously with feedback callback
        send_goal_future: Future = self._action_client.send_goal_async(
            goal=goal_msg,
            feedback_callback=self.feedback_callback,
        )

        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future: Future) -> None:

        self._goal_handle = future.result()

        if not self._goal_handle.accepted:
            self.get_logger().error('Goal was REJECTED by server!')
            return

        self.get_logger().info('Goal ACCEPTED by server. Waiting for result...')
        self._result_future = self._goal_handle.get_result_async()
        self._result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg) -> None:

        feedback = feedback_msg.feedback
        self._feedback_count += 1

        self.get_logger().info(
            f'  [FEEDBACK] Waypoint #{feedback.waypoint_index}: '
            f'({feedback.current_x:.4f}, {feedback.current_y:.4f})'
        )

    def result_callback(self, future: Future) -> None:
        result_response = future.result()
        status = result_response.status
        result = result_response.result


        self.get_logger().info(
            f'RESULT received! Status: {status}\n'
        )

        waypoints_x = result.waypoints_x
        waypoints_y = result.waypoints_y
        num_waypoints = len(waypoints_x)

        if num_waypoints == 0:
            self.get_logger().warn('Result contains NO waypoints!')
            return

        self.get_logger().info(
            f'Total waypoints in result: {num_waypoints}'
        )

        # Print all waypoints in a table
        self.get_logger().info('\nWaypoints table:')
        self.get_logger().info(
            f'  {"Index":>6} | {"X":>12} | {"Y":>12} | {"Dist from prev":>15}'
        )
        prev_x, prev_y = 0.0, 0.0
        for i, (wx, wy) in enumerate(zip(waypoints_x, waypoints_y)):
            if i == 0:
                dist_str = 'START'
            else:
                dist = math.sqrt((wx - prev_x) ** 2 + (wy - prev_y) ** 2)
                dist_str = f'{dist:.4f} m'
            prev_x, prev_y = wx, wy

            label = ''
            if i == 0:
                label = ' <- START'
            elif i == num_waypoints - 1:
                label = ' <- TARGET'

            self.get_logger().info(
                f'  {i:>6} | {wx:>12.4f} | {wy:>12.4f} | '
                f'{dist_str:>15}{label}'
            )

        self._done = True 


def main(args=None):
    rclpy.init(args=args)
    client_node = TrajectoryActionClient()

    while True:
        linear_velocity = float(input("Please write speed: "))
        if linear_velocity == -1:
            break
        target_x = float(input("Coordinate x: "))
        if target_x == 0:
            break
        target_y = float(input("Coordinate y: "))
        if target_y == 0:
            break

        if len(sys.argv) >= 4:
            try:
                linear_velocity = float(sys.argv[1])
                target_x = float(sys.argv[2])
                target_y = float(sys.argv[3])
            except ValueError:
                client_node.get_logger().error(
                    'Неверные аргументы! Использование: action_client <vel> <x> <y>'
                )

        client_node.send_goal(linear_velocity, target_x, target_y)

        # Крутим spin пока не получим результат
        while rclpy.ok() and not client_node._done:
            rclpy.spin_once(client_node, timeout_sec=0.1)  # <-- spin_once с флагом

    # Корректное завершение
    client_node.destroy_node()
    rclpy.shutdown()       


if __name__ == '__main__':
    main()