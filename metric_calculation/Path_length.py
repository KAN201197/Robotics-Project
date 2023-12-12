#!/usr/bin/env python3

import rclpy
from rclpy.qos import QoSProfile
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math

# Initialize variables to store path length and goal position
path_length = 0.0
goal_x = None
goal_y = None
reached_goal = False  # Flag to check if the robot has reached the goal

def path_length_callback(msg):
    global path_length, goal_x, goal_y, reached_goal

    if not reached_goal:
        if len(msg.poses) < 2:
            print("Path contains fewer than 2 poses, cannot calculate path length.")
            return

        for i in range(1, len(msg.poses)):
            x1 = msg.poses[i - 1].pose.position.x
            y1 = msg.poses[i - 1].pose.position.y
            x2 = msg.poses[i].pose.position.x
            y2 = msg.poses[i].pose.position.y

            distance = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
            path_length += distance

        print("Path Length to Goal ({:.2f}, {:.2f}): {:.2f} meters".format(goal_x, goal_y, path_length))

def goal_callback(msg):
    global path_length, goal_x, goal_y, reached_goal

    # Store the goal position when a new goal is received
    goal_x = msg.pose.position.x
    goal_y = msg.pose.position.y

    # Reset path_length for the new goal
    path_length = 0.0
    reached_goal = False

    # Print the goal position
    print("New Goal Position: ({:.2f}, {:.2f})".format(goal_x, goal_y))

def main():
    rclpy.init()
    node = rclpy.create_node('path_length_calculator')
    
    # Define QoS profile to use for the subscriptions
    qos_profile = QoSProfile(depth=10)
    
    path_subscriber = node.create_subscription(Path, '/plan', path_length_callback, qos_profile)
    goal_subscriber = node.create_subscription(PoseStamped, '/goal_pose', goal_callback, qos_profile)

    rclpy.spin(node)

if __name__ == '__main__':
    main()
