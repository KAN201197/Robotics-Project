#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
import numpy as np

class SmoothnessCalculator(Node):
    def __init__(self):
        super().__init__('smoothness_calculator')
        self.subscription_plan = self.create_subscription(Path, '/plan', self.plan_callback, 10)
        self.subscription_odom = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, 10)

        # Initialize variables to store plan and odom data
        self.waypoints = []
        self.robot_positions = []
        
        self.calculate_smoothness_metric()

    def plan_callback(self, msg):
        # Extract waypoints from the plan
        self.waypoints = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]

    def odom_callback(self, msg):
        # Extract robot position from the odometry data
        self.robot_positions.append((msg.pose.pose.position.x, msg.pose.pose.position.y))

    def calculate_smoothness_metric(self):
        if not self.waypoints or not self.robot_positions:
            self.get_logger().warning("No plan or odom data received yet.")
            return None

        if len(self.waypoints) != len(self.robot_positions):
            self.get_logger().warning("Plan and odom data length mismatch.")
            return None

        squared_errors = [np.linalg.norm(np.array(wp) - np.array(rp)) for wp, rp in zip(self.waypoints, self.robot_positions)]
        rmse = np.sqrt(np.mean([err ** 2 for err in squared_errors]))
        return rmse

def main(args=None):
    rclpy.init(args=args)
    smoothness_calculator = SmoothnessCalculator()

    # Set the frequency at which to display the smoothness metric
    display_frequency = 1.0  # Hz

    try:
        while rclpy.ok():
            # Calculate the smoothness metric
            rmse = smoothness_calculator.calculate_smoothness_metric()

            # Display the smoothness metric to the terminal
            print(f"Smoothness Metric (RMSE): {rmse}")

            # Wait for the next display interval
            rclpy.spin_once(smoothness_calculator, timeout_sec=display_frequency)

    except KeyboardInterrupt:
        smoothness_calculator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
