import pytest
import rclpy
import time
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

# Замени на реальное имя пакета и сервиса!
from name_finder_interfaces.srv import FindNames


def find_names_reference(text):
    """Эталонная реализация"""
    return [w for w in text.split() if w and w[0].isupper()]


# --- Unit тесты (без ROS2) ---

def test_empty_string():
    assert find_names_reference('') == []

def test_no_capitals():
    assert find_names_reference('hello world') == []

def test_finds_capitals():
    result = find_names_reference('I went to Moscow yesterday')
    assert 'Moscow' in result
    assert 'I' in result
    assert 'went' not in result

def test_order_preserved():
    result = find_names_reference('Alpha beta Gamma')
    assert result == ['Alpha', 'Gamma']

def test_all_capitals():
    result = find_names_reference('Hello World')
    assert result == ['Hello', 'World']


# --- Integration тесты (нужен запущенный сервер) ---

@pytest.fixture(scope='module')
def client_node():
    rclpy.init()
    node = Node('test_hw3')
    client = node.create_client(FindNames, 'find_names')  # замени имя сервиса!
    yield node, client
    node.destroy_node()
    rclpy.shutdown()


def call_service(node, client, text, timeout=5.0):
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    if not client.wait_for_service(timeout_sec=timeout):
        executor.shutdown()
        return None
    req = FindNames.Request()
    req.text = text
    future = client.call_async(req)
    end = time.time() + timeout
    while not future.done() and time.time() < end:
        executor.spin_once(timeout_sec=0.1)
    executor.shutdown()
    return future.result() if future.done() else None


def test_service_available(client_node):
    node, client = client_node
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    ready = client.wait_for_service(timeout_sec=5.0)
    executor.shutdown()
    assert ready, "Сервис недоступен! Запустите сервер HW3"


def test_empty_returns_empty(client_node):
    node, client = client_node
    resp = call_service(node, client, '')
    assert resp is not None
    assert len(resp.names) == 0


def test_finds_correct_words(client_node):
    node, client = client_node
    text = 'I went to Moscow and Paris yesterday'
    resp = call_service(node, client, text)
    assert resp is not None
    expected = find_names_reference(text)
    assert list(resp.names) == expected


def test_no_capitals_returns_empty(client_node):
    node, client = client_node
    resp = call_service(node, client, 'hello world foo')
    assert resp is not None
    assert len(resp.names) == 0


def test_multiple_calls(client_node):
    node, client = client_node
    cases = [
        ('hello world', []),
        ('Hello World', ['Hello', 'World']),
    ]
    for text, expected in cases:
        resp = call_service(node, client, text)
        assert resp is not None
        assert list(resp.names) == expected