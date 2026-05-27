import pytest
import rclpy
import math
import time
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from trajectory_interfaces.action import Trajectory


def reference_trajectory(velocity, tx, ty):
    dist = math.sqrt(tx ** 2 + ty ** 2)
    if dist < 1e-9:
        return [(0.0, 0.0)]
    dx, dy = tx/dist, ty/dist
    step = velocity * 1.0
    pts = [(0.0, 0.0)]
    cx, cy = 0.0, 0.0
    while True:
        nx, ny = cx + step*dx, cy + step*dy
        if math.sqrt(nx ** 2 + ny ** 2) > dist:
            break
        cx, cy = nx, ny
        pts.append((cx, cy))
    pts.append((tx, ty))
    return pts


# --- Unit тесты (без ROS2) ---

def test_starts_at_origin():
    pts = reference_trajectory(1.5, 5.0, 5.0)
    assert pts[0] == (0.0, 0.0)

def test_ends_at_target():
    pts = reference_trajectory(1.5, 6.0, 8.0)
    assert pts[-1] == (6.0, 8.0)

def test_step_size():
    velocity = 2.0
    pts = reference_trajectory(velocity, 10.0, 0.0)
    for i in range(1, len(pts)-1):
        d = math.sqrt((pts[i][0]-pts[i-1][0]) ** 2 + (pts[i][1]-pts[i-1][1]) ** 2)
        assert d == pytest.approx(velocity, abs=1e-6)

def test_no_overshoot():
    tx, ty = 5.0, 5.0
    total = math.sqrt(tx ** 2 + ty ** 2)
    pts = reference_trajectory(1.5, tx, ty)
    for p in pts[:-1]:
        assert math.sqrt(p[0] ** 2 + p[1]** 2) <= total + 1e-6

def test_min_two_points():
    pts = reference_trajectory(100.0, 1.0, 0.0)
    assert len(pts) == 2

def test_example_5_5():
    """velocity=1.5, target=(5,5) -> 6 точек"""
    pts = reference_trajectory(1.5, 5.0, 5.0)
    assert len(pts) == 6


# --- Integration тесты (нужен запущенный сервер) ---

@pytest.fixture(scope='module')
def action_node():
    rclpy.init()
    node = Node('test_hw4')
    client = ActionClient(node, Trajectory, 'trajectory_planner')
    yield node, client
    node.destroy_node()
    rclpy.shutdown()


def send_goal(node, client, vel, tx, ty, timeout=15.0):
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    if not client.wait_for_server(timeout_sec=timeout):
        executor.shutdown()
        return None, []

    goal = Trajectory.Goal()
    goal.linear_velocity = float(vel)
    goal.target_x = float(tx)
    goal.target_y = float(ty)

    feedback_list = []
    future = client.send_goal_async(
        goal,
        feedback_callback=lambda f: feedback_list.append(f.feedback)
    )

    end = time.time() + timeout
    while not future.done() and time.time() < end:
        executor.spin_once(timeout_sec=0.1)

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


def test_server_available(action_node):
    node, client = action_node
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    ready = client.wait_for_server(timeout_sec=5.0)
    executor.shutdown()
    assert ready, "Action сервер недоступен! Запустите action_server"


def test_feedback_received(action_node):
    node, client = action_node
    _, feedback = send_goal(node, client, 1.5, 5.0, 5.0)
    assert len(feedback) > 0


def test_no_overshoot_integration(action_node):
    node, client = action_node
    tx, ty = 5.0, 5.0
    total = math.sqrt(tx ** 2 + ty ** 2)
    result, _ = send_goal(node, client, 1.5, tx, ty)
    assert result is not None
    xs, ys = result.waypoints_x, result.waypoints_y
    for i in range(len(xs)-1):
        assert math.sqrt(xs[i] ** 2 + ys[i] ** 2) <= total + 1e-6