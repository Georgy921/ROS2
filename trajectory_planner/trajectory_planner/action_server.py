import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

import math
import time

from trajectory_interfaces.action import Trajectory


class TrajectoryActionServer(Node):
    def __init__(self):
        super().__init__('trajectory_action_server')

        # Use ReentrantCallbackGroup to allow concurrent goal processing
        self._callback_group = ReentrantCallbackGroup()

        # Create action server
        self._action_server = ActionServer(
            node=self,
            action_type=Trajectory,
            action_name='trajectory_planner',
            execute_callback=self.execute_callback,
            callback_group=self._callback_group,
        )

        self.get_logger().info(
            '=' * 50 + '\n'
            'Trajectory Action Server is ready!\n'
            'Action: /trajectory_planner\n'
            + '=' * 50
        )

    def execute_callback(self, goal_handle: ServerGoalHandle):
        """Main execution callback - builds trajectory and sends feedback."""

        # Extract goal parameters
        velocity = goal_handle.request.linear_velocity
        target_x = goal_handle.request.target_x
        target_y = goal_handle.request.target_y

        self.get_logger().info(
            f'\n{"=" * 40}\n'
            f'New goal received!\n'
            f'  Velocity: {velocity:.3f} m/s\n'
            f'  Target: ({target_x:.3f}, {target_y:.3f})\n'
            f'{"=" * 40}'
        )

        # --- Input validation ---
        if velocity <= 0.0:
            self.get_logger().error(
                f'Invalid velocity: {velocity}. Must be positive!'
            )
            goal_handle.abort()
            result = Trajectory.Result()
            result.waypoints_x = []
            result.waypoints_y = []
            return result

        # Compute total distance to target
        total_distance = math.sqrt(target_x ** 2 + target_y ** 2)

        if total_distance < 1e-9:
            self.get_logger().warn(
                'Target point is at origin (0,0)! '
                'Returning single point.'
            )
            goal_handle.succeed()
            result = Trajectory.Result()
            result.waypoints_x = [0.0]
            result.waypoints_y = [0.0]
            return result

        # --- Compute direction unit vector ---
        dir_x = target_x / total_distance
        dir_y = target_y / total_distance

        # Step size = distance covered in 1 second at given velocity
        step_size = velocity * 1.0  # [m/s] * [s] = [m]

        self.get_logger().info(
            f'  Total distance: {total_distance:.3f} m\n'
            f'  Step size:      {step_size:.3f} m\n'
            f'  Direction:      ({dir_x:.4f}, {dir_y:.4f})'
        )

        # --- Initialize trajectory with start point (0, 0) ---
        waypoints_x = [0.0]
        waypoints_y = [0.0]

        # Current position starts at origin
        current_x = 0.0
        current_y = 0.0

        # Send feedback for start point
        feedback_msg = Trajectory.Feedback()
        feedback_msg.current_x = current_x
        feedback_msg.current_y = current_y
        feedback_msg.waypoint_index = 0
        goal_handle.publish_feedback(feedback_msg)

        self.get_logger().info(
            f'  Waypoint [0]: ({current_x:.4f}, {current_y:.4f}) - START'
        )
        # --- Build intermediate waypoints ---
        waypoint_index = 1

        while True:
            # Check for cancellation
            if goal_handle.is_cancel_requested:
                self.get_logger().warn('Goal was cancelled!')
                goal_handle.canceled()
                result = Trajectory.Result()
                result.waypoints_x = waypoints_x
                result.waypoints_y = waypoints_y
                return result

            # Compute next candidate waypoint
            next_x = current_x + step_size * dir_x
            next_y = current_y + step_size * dir_y

            # --- Overshoot check ---
            # Distance from NEXT point to origin
            dist_next_from_origin = math.sqrt(next_x ** 2 + next_y ** 2)

            if dist_next_from_origin > total_distance:
                # Next waypoint overshoots target - stop here
                self.get_logger().info(
                    f'  Overshoot detected at step {waypoint_index}:\n'
                    f'    Next point: ({next_x:.4f}, {next_y:.4f})\n'
                    f'    Distance from origin: {dist_next_from_origin:.4f} m\n'
                    f'    Total distance: {total_distance:.4f} m\n'
                    f'  Stopping loop.'
                )
                break

            # Accept this waypoint
            current_x = next_x
            current_y = next_y

            waypoints_x.append(current_x)
            waypoints_y.append(current_y)

            # Publish feedback with current waypoint
            feedback_msg = Trajectory.Feedback()
            feedback_msg.current_x = current_x
            feedback_msg.current_y = current_y
            feedback_msg.waypoint_index = waypoint_index
            goal_handle.publish_feedback(feedback_msg)

            self.get_logger().info(
                f'  Waypoint [{waypoint_index}]: '
                f'({current_x:.4f}, {current_y:.4f})'
            )

            waypoint_index += 1

            # Small delay to make feedback visible
            time.sleep(0.3)

        # --- Always add target point as final waypoint ---
        waypoints_x.append(target_x)
        waypoints_y.append(target_y)

        # Send feedback for final target point
        feedback_msg = Trajectory.Feedback()
        feedback_msg.current_x = target_x
        feedback_msg.current_y = target_y
        feedback_msg.waypoint_index = waypoint_index
        goal_handle.publish_feedback(feedback_msg)

        self.get_logger().info(
            f'  Waypoint [{waypoint_index}]: '
            f'({target_x:.4f}, {target_y:.4f}) - TARGET\n'
            f'  Total waypoints: {len(waypoints_x)}'
        )

        # --- Build and return result ---
        result = Trajectory.Result()
        result.waypoints_x = waypoints_x
        result.waypoints_y = waypoints_y

        goal_handle.succeed()

        self.get_logger().info(
            f'\n{"=" * 40}\n'
            f'Goal succeeded!\n'
            f'  Total waypoints in result: {len(waypoints_x)}\n'
            f'{"=" * 40}'
        )

        return result


def main(args=None):
    rclpy.init(args=args)

    server_node = TrajectoryActionServer()

    # MultiThreadedExecutor allows concurrent callbacks
    executor = MultiThreadedExecutor()
    executor.add_node(server_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        server_node.get_logger().info('Server shutting down...')
    finally:
        server_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()