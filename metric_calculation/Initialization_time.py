#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
import time

class InitTimeCalculator(Node):
    def __init__(self):
        super().__init__('init_time_calculator')

        # Create a subscriber to the global path topic
        self.subscription = self.create_subscription(
            Path,
            '/plan',  # Replace with the actual topic name
            self.path_callback,
            10  # Adjust the queue size as needed
        )
        self.subscription  # prevent unused variable warning

        # Initialize variables
        self.start_time = None

    def path_callback(self, msg):
        if self.start_time is None:
            # Record the start time when the first path message is received
            self.start_time = time.time()

            # Calculate the initialization time
            initialization_time = time.time() - self.start_time

            # Print the initialization time
            self.get_logger().info(f'Initialization Time: {initialization_time} seconds')

def main(args=None):
    rclpy.init(args=args)
    init_time_calculator = InitTimeCalculator()
    rclpy.spin(init_time_calculator)
    init_time_calculator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
