#!/usr/bin/env python3

import rclpy
from rclpy.qos import QoSProfile
from nav_msgs.msg import Path
from builtin_interfaces.msg import Time

# Initialize variables to store path planning start time and sum of planning times
path_planning_start_time = None
total_path_planning_time = 0
num_path_plans = 0

def path_planning_callback(msg):
    global path_planning_start_time, total_path_planning_time, num_path_plans

    if len(msg.poses) < 2:
        print("Path contains fewer than 2 poses, cannot calculate path planning time.")
        return

    if path_planning_start_time is None:
        # Store the start time when a new path message is received
        path_planning_start_time = msg.header.stamp
        print("Path planning started at {:.2f} seconds".format(path_planning_start_time.sec + path_planning_start_time.nanosec / 1e9))
    else:
        # Calculate the path planning time
        end_time = msg.header.stamp
        time_diff_sec = end_time.sec - path_planning_start_time.sec
        time_diff_nsec = end_time.nanosec - path_planning_start_time.nanosec
        path_planning_time = time_diff_sec + time_diff_nsec / 1e9
        print("Path Planning Time: {:.2f} seconds".format(path_planning_time))

        # Update total time and count
        total_path_planning_time += path_planning_time
        num_path_plans += 1

        # Calculate and display average path planning time
        if num_path_plans > 0:
            avg_path_planning_time = total_path_planning_time / num_path_plans
            print("Average Path Planning Time: {:.2f} seconds".format(avg_path_planning_time))

        # Reset start time for the next plan
        path_planning_start_time = end_time

def main():
    global path_planning_start_time, total_path_planning_time, num_path_plans

    rclpy.init()
    node = rclpy.create_node('path_planning_time_calculator')
    
    # Define QoS profile to use for the subscription
    qos_profile = QoSProfile(depth=10)
    
    path_subscriber = node.create_subscription(Path, '/plan', path_planning_callback, qos_profile)

    rclpy.spin(node)

if __name__ == '__main__':
    main()
