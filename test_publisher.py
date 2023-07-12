import sys
import unittest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from data_processor.publisher import DataPublisher
from data_processor.msg import Data

class TestPublisher(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.executor = SingleThreadedExecutor()
        self.publisher = DataPublisher()
        self.executor.add_node(self.publisher)
        self.received_msgs = []

    def tearDown(self):
        self.executor.shutdown()
        self.publisher.destroy_node()
        rclpy.shutdown()

    def test_published_data(self):
        def callback(msg):
            self.received_msgs.append(msg)
        self.publisher.publisher.create_dependency(callback)

        for _ in range(3):  # Execute callbacks for 3 messages
            self.executor.spin_once(timeout_sec=0.5)

        self.assertEqual(len(self.received_msgs), 3)

        expected_latitudes = [41.233283, 41.233209, 41.233247]
        received_latitudes = [msg.latitude for msg in self.received_msgs]
        self.assertEqual(expected_latitudes, received_latitudes)

if __name__ == '__main__':
    unittest.main(argv=['first-arg-is-ignored'], exit=False)