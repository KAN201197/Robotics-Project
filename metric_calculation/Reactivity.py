#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
import time
import math
import signal

class ReactivityMetricCalculator(Node):
    def __init__(self):
        super().__init__('reactivity_metric_calculator')
        self.subscription_local = self.create_subscription(
            Path,
            '/local_plan', 
            self.local_path_callback,
            10)
        self.subscription_global = self.create_subscription(
            Path,
            '/plan', 
            self.global_path_callback,
            10)
        self.last_local_path_timestamp = None
        self.last_global_path = None
        self.reactivity_metrics = []  # List to store individual reactivity metric values
        self.num_samples = 0  # Counter for the number of samples

        # Set up a signal handler to calculate and log the average on SIGINT (Ctrl+C)
        signal.signal(signal.SIGINT, self.handle_sigint)

    def local_path_callback(self, msg):
        # Record the timestamp when a new local path is received
        current_time = time.time()
        if self.last_local_path_timestamp is not None:
            time_delay = current_time - self.last_local_path_timestamp

            # Assuming you have access to the desired global path
            if self.last_global_path is not None:
                distance_to_global_path = calculate_distance_to_global_path(
                    msg, self.last_global_path)
                # Compute the reactivity metric based on time delay and distance
                reactivity_metric = compute_reactivity_metric(
                    time_delay, distance_to_global_path)
                self.reactivity_metrics.append(reactivity_metric)  # Store the individual metric
                self.num_samples += 1  # Increment the sample counter
                self.get_logger().info(f'Reactivity Metric: {reactivity_metric}')
        self.last_local_path_timestamp = current_time

    def global_path_callback(self, msg):
        # Update the desired global path
        self.last_global_path = msg
    
    def handle_sigint(self, signum, frame):
        # Calculate the average reactivity metric
        average_reactivity_metric = 0.0
        if self.num_samples > 0:
            average_reactivity_metric = sum(self.reactivity_metrics) / self.num_samples

        self.get_logger().info(f'Average Reactivity Metric: {average_reactivity_metric}')

def calculate_distance_to_global_path(local_path, global_path):
    # Calculate the distance between the last point of the local path
    # and the nearest point on the global path
    if not local_path.poses or not global_path.poses:
        return float('inf')

    local_end_point = local_path.poses[-1]
    min_distance = float('inf')

    for global_point in global_path.poses:
        distance = euclidean_distance(local_end_point.pose.position, global_point.pose.position)
        min_distance = min(min_distance, distance)

    return min_distance

def compute_reactivity_metric(time_delay, distance_to_global_path):
    # A simple reactivity metric as a combination of time delay and distance
    return time_delay + 1 * distance_to_global_path  # Adjust weights as needed

def euclidean_distance(point1, point2):
    return math.sqrt((point1.x - point2.x)**2 + (point1.y - point2.y)**2)

def main(args=None):
    rclpy.init(args=args)
    reactivity_metric_calculator = ReactivityMetricCalculator()
    
    try:
        rclpy.spin(reactivity_metric_calculator)
    except KeyboardInterrupt:
        pass

    reactivity_metric_calculator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    