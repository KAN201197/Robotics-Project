#!/usr/bin/env python3

import rclpy
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry, OccupancyGrid  
import time
import math
import csv 
import matplotlib.pyplot as plt

# Initialize variables to store path length and goal position
path_length = 0.0
goal_x = None
goal_y = None
reached_goal = False  # Flag to check if the robot has reached the goal

# Initialize variables for the robot's current pose
robot_x = None
robot_y = None

# Clearance metric related variables
clearance = 0.0
min_clearance = None
costmap = None
robot_radius = 0.3  # Adjust as needed

def costmap_callback(msg):
    global costmap, robot_x, robot_y, clearance, min_clearance
    costmap = msg

    # Get the position in the costmap grid
    x = int((robot_x - costmap.info.origin.position.x) / costmap.info.resolution)
    y = int((robot_y - costmap.info.origin.position.y) / costmap.info.resolution)

    if x >= 0 and y >= 0 and x < costmap.info.width and y < costmap.info.height:
        clearance = calculate_clearance(costmap, x, y, robot_x, robot_y, robot_radius, costmap.info.resolution)
        if min_clearance is None or clearance < min_clearance or clearance != 0:
            min_clearance = clearance
    else:
        clearance = 0.0 
    
    print(f'Clearance: {clearance}')

def odom_callback(msg):
    global robot_x, robot_y
    robot_x = msg.pose.pose.position.x
    robot_y = msg.pose.pose.position.y

def calculate_clearance(costmap, x, y, robot_x, robot_y, robot_radius, resolution):
    # Calculate the clearance using ray casting
    clearance = robot_radius 

    for angle_deg in range(0, 360, 5):  
        angle_rad = math.radians(angle_deg)
        step_size = resolution  

        for distance in range(0, int(robot_radius / resolution)):
            x_ray = int(x + distance * math.cos(angle_rad))
            y_ray = int(y + distance * math.sin(angle_rad))

            if (
                x_ray >= 0 and x_ray < costmap.info.width and
                y_ray >= 0 and y_ray < costmap.info.height
            ):
                cell_value = costmap.data[y_ray * costmap.info.width + x_ray]
                if cell_value != 0:
                    # Obstacle detected, update clearance
                    clearance = min(clearance, distance * resolution)
                    break

    return clearance

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
    global path_length, goal_x, goal_y, reached_goal, clearance

    # Store the goal position when a new goal is received
    goal_x = msg.pose.position.x
    goal_y = msg.pose.position.y

    # Reset path_length for the new goal
    path_length = 0.0
    reached_goal = False

    # Print the goal position
    print("New Goal Position: ({:.2f}, {:.2f})".format(goal_x, goal_y))

    # Set reached_goal flag to False when a new goal is received
    reached_goal = False

    # Initialize clearance for the new goal
    clearance = 0.0

# Calculate the Euclidean distance between two positions
def calculate_distance(x1, y1, x2, y2):
    dx = x1 - x2
    dy = y1 - y2
    return math.sqrt(dx * dx + dy * dy)

# Create a function to save metrics to a CSV file
def save_metrics_to_csv(filename, metrics):
    with open(filename, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['Metric', 'Value'])
        for metric, value in metrics.items():
            writer.writerow([metric, value])

# Function to create a bar plot
def create_bar_plot(metrics):
    metric_names = list(metrics.keys())
    metric_values = list(metrics.values())

    plt.figure(figsize=(10, 6))
    plt.bar(metric_names, metric_values, color=['blue', 'green', 'red', 'purple'])
    plt.xlabel('Metric')
    plt.ylabel('Value')
    plt.title('Performance Metrics')
    plt.xticks(rotation=45)
    plt.tight_layout()
    plt.show()

def main():
    rclpy.init()

    navigator = BasicNavigator()

    # Set our demo's initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = -10.5
    initial_pose.pose.position.y = 8.3
    navigator.setInitialPose(initial_pose)

    # Record the start time for initialization
    initialization_start_time = time.time()

    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active()

    # Record the end time for initialization
    initialization_end_time = time.time()

    # Calculate initialization time
    initialization_time = initialization_end_time - initialization_start_time
    print(f'Initialization time: {initialization_time:.2f} seconds')

    # Subscribe to the robot's odometry topic to get its pose
    odom_subscription = navigator.create_subscription(
        Odometry,
        '/odometry/filtered', 
        odom_callback,
        10  # QoS profile depth
    )

    # Subscribe to the costmap topic to obtain occupancy grid data
    costmap_subscription = navigator.create_subscription(
        OccupancyGrid,
        '/global_costmap/costmap',  # Replace with your costmap topic
        costmap_callback,
        10  # QoS profile depth
    )

    # Wait until you receive the costmap data before initiating navigation
    while rclpy.ok() and costmap is None:
        rclpy.spin_once(navigator)

    # Now that you have the costmap data, you can proceed with navigation

    # Record the start time for path planning
    path_planning_start_time = time.perf_counter_ns()

    # Go to our demo's first goal pose
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = 11.0
    goal_pose.pose.position.y = -13.0
    goal_pose.pose.orientation.w = 1.0

    path = navigator.getPath(initial_pose, goal_pose)

    navigator.goToPose(goal_pose)

    # Record the end time for path planning
    path_planning_end_time = time.perf_counter_ns()

    # Calculate path planning time in nanoseconds
    path_planning_time_ns = path_planning_end_time - path_planning_start_time

    # Convert nanoseconds to seconds
    path_planning_time = path_planning_time_ns / 1e9
    print(f'Path planning start time: {path_planning_start_time} ns')
    print(f'Path planning end time: {path_planning_end_time} ns')
    print(f'Path planning time: {path_planning_time:.2f} seconds')

    # Record the start time for execution
    execution_start_time = time.time()

    i = 0
    visited_poses = []
    total_length = 0.0
    prev_x, prev_y = None, None  # Initialize the previous position
    while not navigator.isTaskComplete():
        # Calculate and print execution time
        current_time = time.time()
        execution_time = current_time - execution_start_time
        print(f'Execution time: {execution_time:.2f} seconds')

        # Append the current position to the list
        visited_poses.append((robot_x, robot_y))

        # Calculate the distance traveled based on consecutive positions
        if prev_x is not None and prev_y is not None:
            distance = calculate_distance(prev_x, prev_y, robot_x, robot_y)
            total_length += distance

        prev_x, prev_y = robot_x, robot_y

    # Record the end time for execution again
    execution_end_time = time.time()

    # Calculate execution time after the task is complete
    execution_time = execution_end_time - execution_start_time
    print(f'Total Execution time: {execution_time:.2f} seconds')

    # Calculate and print the total path length
    print(f'Total path length: {total_length:.2f} meters')

    print(f'Minimum clearance: {min_clearance}')

    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    # Collect metrics
    metrics = {
        'Initialization Time (s)': initialization_time,
        'Path Planning Time (s)': path_planning_time,
        'Total Execution Time (s)': execution_time,
        'Total Path Length (m)': total_length,
        'Minimum Clearance': min_clearance,  
    }

    # Save metrics to a CSV file
    save_metrics_to_csv('metrics.csv', metrics)

    # Create a bar plot of the metrics
    create_bar_plot(metrics)

if __name__ == '__main__':
    main()
