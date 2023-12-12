#!/usr/bin/env python3

import math
import numpy as np
import rclpy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import csv
import matplotlib.pyplot as plt

# Define a class to hold the smoothness metrics and count
class SmoothnessMetricsAccumulator:
    def __init__(self):
        self.total_smoothness = 0.0
        self.num_paths = 0
        self.last_average_smoothness = 0.0  

    def add_smoothness(self, smoothness):
        self.total_smoothness += smoothness
        self.num_paths += 1

    def calculate_average(self):
        if self.num_paths == 0:
            return 0.0  
        average_smoothness = self.total_smoothness / self.num_paths
        self.last_average_smoothness = average_smoothness 
        return average_smoothness

smoothness_accumulator = SmoothnessMetricsAccumulator()

# Create lists to store smoothness metrics and indices
smoothness_values = []
indices = []

def calculate_smoothness(path):
    smoothness = 0.0
    prev_yaw = None

    for pose in path.poses:
        if prev_yaw is not None:
            # Calculate the change in yaw angle
            current_yaw = math.atan2(2.0 * (pose.pose.orientation.w * pose.pose.orientation.z + pose.pose.orientation.x * pose.pose.orientation.y),
                                     1.0 - 2.0 * (pose.pose.orientation.y**2 + pose.pose.orientation.z**2))
            yaw_diff = abs(current_yaw - prev_yaw)

            # Use yaw difference as a measure of smoothness (smaller differences imply smoother path)
            smoothness += yaw_diff

        prev_yaw = math.atan2(2.0 * (pose.pose.orientation.w * pose.pose.orientation.z + pose.pose.orientation.x * pose.pose.orientation.y),
                              1.0 - 2.0 * (pose.pose.orientation.y**2 + pose.pose.orientation.z**2))

    return smoothness

def path_callback(msg):
    # Calculate smoothness when a new path is received
    smoothness_metric = calculate_smoothness(msg)
    print(f"Smoothness Metric: {smoothness_metric}")
    
    # Add the smoothness metric to the accumulator
    smoothness_accumulator.add_smoothness(smoothness_metric)

    # Append smoothness metric to the list and increment the index
    smoothness_values.append(smoothness_metric)
    indices.append(len(smoothness_values) - 1)  

    # Calculate and print the average smoothness metric
    average_smoothness = smoothness_accumulator.calculate_average()
    print(f"Average Smoothness Metric: {average_smoothness}")

def save_smoothness_data_to_csv(x, y):
    with open('smoothness_data.csv', 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['Index', 'Smoothness'])
        for i, smoothness in zip(x, y):
            writer.writerow([i, smoothness])

def plot_smoothness_data(x, y):
    plt.figure()
    plt.plot(x, y, 'bo-', label='Smoothness')
    plt.xlabel('Data Points')
    plt.ylabel('Smoothness Metric')
    plt.title('Smoothness Metric')
    plt.legend()
    plt.grid(True)
    plt.show()

def main():
    rclpy.init()
    node = rclpy.create_node('smoothness_calculator')

    # Subscribe to the /plan topic
    path_subscription = node.create_subscription(Path, '/plan', path_callback, 10)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Save smoothness data to a CSV file
        save_smoothness_data_to_csv(indices, smoothness_values)
        # Plot the smoothness data
        plot_smoothness_data(indices, smoothness_values)

        node.destroy_node()
        rclpy.shutdown()
    
if __name__ == "__main__":
    main()