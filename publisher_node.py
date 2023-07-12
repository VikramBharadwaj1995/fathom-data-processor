import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from data_processor.msg import Data
import csv
import time

class DataPublisher(Node):
    def __init__(self):
        super().__init__('data_publisher')
        qos = QoSProfile(depth=10)
        self.publisher = self.create_publisher(Data, 'data', qos)
        self.timer = self.create_timer(1.0/3.0, self.publish_data)
        self.csv_file = './data.csv'
        self.csv_data = self.read_csv()

    def read_csv(self):
        with open(self.csv_file, 'r') as csvfile:
            reader = csv.DictReader(csvfile)
            return list(reader)

    def publish_data(self):
        if not self.csv_data:
            self.destroy_timer(self.timer)
            return

        row = self.csv_data.pop(0)
        msg = Data()
        msg.longitude = float(row['Longitude'])
        msg.latitude = float(row['Latitude'])
        msg.altitude = float(row['Altitude'])
        msg.time = row['Time']
        msg.actual_speed = float(row['Actual_Speed'])

        self.publisher.publish(msg)
        self.get_logger().info('Published: %s' % msg.time)

def main(args=None):
    rclpy.init(args=args)
    data_publisher = DataPublisher()
    rclpy.spin(data_publisher)
    data_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()