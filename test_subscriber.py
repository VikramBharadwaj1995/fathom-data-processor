import sys
import unittest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from data_processor.subscriber import DataSubscriber
from data_processor.msg import Data
from std_msgs.msg import Float64

class TestSubscriber(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.executor = SingleThreadedExecutor()
        self.subscriber = DataSubscriber()
        self.executor.add_node(self.subscriber)
        self.received_msgs = []

    def tearDown(self):
        self.executor.shutdown()
        self.subscriber.destroy_node()
        rclpy.shutdown()

    def test_time_difference(self):
        def callback(msg):
            self.received_msgs.append(msg)
        self.subscriber.publisher.create_dependency(callback)

        pub = self.subscriber.create_publisher(Data, 'data', 10)

        def send_data(data):
            msg = Data()
            msg.longitude, msg.latitude, msg.altitude, msg.time, msg.actual_speed = data
            pub.publish(msg)

        data1 = (-105.434187, 41.233283, 2606.7, '2014-07-26 13:03:55+00:00', 5.1)
        data2 = (-105.434177, 41.233209, 2607.6, '2014-07-26 13:03:58+00:00', 5.3)
        data3 = (-105.434183, 41.233247, 2607.2, '2014-07-26 13:04:15+00:00', 5.5)

        send_data(data1)
        self.executor.spin_once(timeout_sec=0.5)
        send_data(data2)
        self.executor.spin_once(timeout_sec=0.5)
        send_data(data3)
        self.executor.spin_once(timeout_sec=0.5)

        self.assertEqual(len(self.received_msgs), 2)
        self.assertEqual(self.received_msgs[0].data, 3.0)
        self.assertEqual(self.received_msgs[1].data, 17.0)

if __name__ == '__main__':
    unittest.main(argv=['first-arg-is-ignored'], exit=False)