import pytest
import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import Int64
from rclpy.executors import SingleThreadedExecutor


def largest_prime_factor(n):
    if n <= 1:
        return n
    largest = 1
    while n % 2 == 0:
        largest = 2
        n //= 2
    f = 3
    while f * f <= n:
        while n % f == 0:
            largest = f
            n //= f
        f += 2
    if n > 1:
        largest = n
    return largest


# --- Unit тесты (без ROS2) ---

def test_lpf_28():
    """28 = 2*2*7, ответ 7"""
    assert largest_prime_factor(28) == 7

def test_lpf_12():
    assert largest_prime_factor(12) == 3

def test_lpf_prime():
    assert largest_prime_factor(17) == 17

def test_lpf_100():
    assert largest_prime_factor(100) == 5

def test_lpf_result_is_prime():
    def is_prime(n):
        if n < 2:
            return False
        for i in range(2, int(n**0.5)+1):
            if n % i == 0:
                return False
        return True
    for n in range(2, 50):
        assert is_prime(largest_prime_factor(n))


# --- Integration тесты (нужны запущенные узлы) ---

@pytest.fixture(scope='module')
def node():
    rclpy.init()
    n = Node('test_hw2')
    yield n
    n.destroy_node()
    rclpy.shutdown()


def collect_msgs(node, topic, count=5, timeout=8.0):
    msgs = []
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    sub = node.create_subscription(
        Int64, topic, lambda m: msgs.append(m.data), 10
    )
    end = time.time() + timeout
    while len(msgs) < count and time.time() < end:
        executor.spin_once(timeout_sec=0.1)
    node.destroy_subscription(sub)
    executor.shutdown()
    return msgs


def test_input_topic_receives_numbers(node):
    """Топик с числами должен публиковать натуральные числа"""
    msgs = collect_msgs(node, '/numbers')
    assert len(msgs) > 0
    assert all(m > 0 for m in msgs)




