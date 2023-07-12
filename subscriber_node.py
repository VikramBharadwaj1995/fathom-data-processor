import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from data_processor.msg import Data
from std_msgs.msg import Float64

from dateutil.parser import parse

class DataSubscriber(Node):
    def __init__(self):
        super().__init__('data_subscriber')
        qos = QoSProfile(depth=10)
        self.subscriber = self.create_subscription(Data, 'data', self.data_callback, qos)
        self.publisher = self.create_publisher(Float64, 'diff', qos)
        self.prev_time = None

    def data_callback(self, msg):
        if self.prev_time:
            time_diff = (parse(msg.time) - self.prev_time).total_seconds()
            diff_msg = Float64()
            diff_msg.data = time_diff
            self.publisher.publish(diff_msg)
            self.get_logger().info('Published time difference: %.1f' % time_diff)

        self.prev_time = parse(msg.time)

def main(args=None):
    rclpy.init(args=args)
    data_subscriber = DataSubscriber()
    rclpy.spin(data_subscriber)
    data_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()