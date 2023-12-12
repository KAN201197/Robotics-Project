#!/usr/bin/env python3

import rclpy
from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry
import time

# Initialize variables to store the execution start time and flag to detect movement stop
execution_start_time = None
robot_moving = False

# Threshold for detecting near-zero velocity (you may need to adjust this)
VELOCITY_THRESHOLD = 0.01  # m/s

def odom_callback(msg):
    global execution_start_time, robot_moving

    # Calculate the robot's linear velocity magnitude
    velocity = msg.twist.twist.linear.x

    # Check if the robot is moving (linear velocity > threshold)
    if velocity > VELOCITY_THRESHOLD:
        robot_moving = True
    else:
        # If the robot was moving but now has stopped, calculate and print execution time
        if robot_moving and execution_start_time is not None:
            execution_time = time.time() - execution_start_time
            print("Execution Time: {:.2f} seconds".format(execution_time))
            execution_start_time = None
            robot_moving = False
    # Start measuring execution time when the robot starts moving
    if robot_moving and execution_start_time is None:
        execution_start_time = time.time()

def main():
    global execution_start_time

    rclpy.init()
    node = rclpy.create_node('execution_time_calculator')

    # Define QoS profile to use for the subscription
    qos_profile = QoSProfile(depth=10)

    odom_subscriber = node.create_subscription(Odometry, '/odometry/filtered', odom_callback, qos_profile)

    rclpy.spin(node)

if __name__ == '__main__':
    main()

