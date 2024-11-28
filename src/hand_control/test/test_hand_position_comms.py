import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from rclpy.executors import SingleThreadedExecutor
import pytest
import time

@pytest.mark.rostest
def test_hand_position_state_communication():
    # Initialize ROS2
    rclpy.init()

    # Create a test node
    class HandPositionTestNode(Node):
        def __init__(self):
            super().__init__('hand_position_test_node')
            self.received_data = None
            self.subscriber = self.create_subscription(
                Float32,
                '/hand_position_state',
                self.listener_callback,
                10
            )

        def listener_callback(self, msg):
            self.get_logger().info(f"Received message: {msg.data}")
            self.received_data = msg.data

    # Test Variables
    test_node = HandPositionTestNode()
    publisher_node = Node('hand_position_test_publisher')
    publisher = publisher_node.create_publisher(Float32, '/hand_position_state', 10)

    # ROS2 Executor
    executor = SingleThreadedExecutor()
    executor.add_node(test_node)
    executor.add_node(publisher_node)

    try:
        # Publish a fake hand position state
        test_data = 5.0
        msg = Float32()
        msg.data = test_data

        for _ in range(3):  # Publish multiple times for reliability
            publisher.publish(msg)
            time.sleep(0.1)

        # Allow some time for the subscriber to process the message
        timeout = time.time() + 2.0  # Wait up to 2 seconds
        while test_node.received_data is None and time.time() < timeout:
            time.sleep(0.1)

        # Verify that the subscriber received the correct message
        assert test_node.received_data == test_data, "The subscriber did not receive the expected data"

    finally:
        # Shutdown nodes and executor
        executor.shutdown()
        test_node.destroy_node()
        publisher_node.destroy_node()
        rclpy.shutdown()
