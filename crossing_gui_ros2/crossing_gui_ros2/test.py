

import rclpy
from rclpy.node import Node
import time

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')

    def my_function(self):
        while rclpy.ok():  # Check if ROS is still running
            self.get_logger().info('Hello ROS 2!')
            time.sleep(1.0)  # Sleep for 1 seconds

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    node.my_function()  # Call the member function with the while loop
    rclpy.shutdown()

if __name__ == '__main__':
    main()