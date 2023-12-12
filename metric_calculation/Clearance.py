#!/usr/bin/env python3

import rclpy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
import math
from geometry_msgs.msg import PoseStamped
import csv
import matplotlib.pyplot as plt

def costmap_callback(msg):
    global costmap_data
    costmap_data = msg
    global global_resolution
    global_resolution = msg.info.resolution

    # Check if both costmap and global path data are available
    if costmap_data is not None and global_path is not None:
        # Start the timer for calculating clearance
        start_clearance_timer()

def global_path_callback(msg):
    global global_path
    global_path = msg.poses

    # Check if both costmap and global path data are available
    if costmap_data is not None and global_path is not None:
        # Start the timer for calculating clearance
        start_clearance_timer()

def start_clearance_timer():
    global clearance_timer

    # Ensure the timer is not already created
    if clearance_timer is None:
        clearance_timer = node.create_timer(1.0, lambda: calculate_clearance(global_path, costmap_data, global_resolution))

def calculate_clearance(path, costmap, resolution):
    total_clearance = 0.0
    count = 0
    robot_radius = 0.3

    # Use an index to keep track of data points
    index = 0

    for pose in path:
        # Get the position in the costmap grid
        x = int((pose.pose.position.x - costmap.info.origin.position.x) / resolution)
        y = int((pose.pose.position.y - costmap.info.origin.position.y) / resolution)

        if x < 0 or y < 0 or x >= costmap.info.width or y >= costmap.info.height:
            continue

        # Calculate the distance from the robot's current pose to the nearest obstacle
        clearance = calculate_ray_casting_clearance(costmap, x, y, robot_radius, resolution)

        total_clearance += clearance
        count += 1

        # Append clearance value and timestamp to the lists
        clearance_values.append(clearance)
        indices.append(index)
        index += 1
        timestamps.append(node.get_clock().now().to_msg().sec)

    if count == 0:
        average_clearance = 0.0  # Avoid division by zero
    else:
        average_clearance = total_clearance / count

    # Print the average clearance metric result
    node.get_logger().info(f"Average Clearance Metric: {average_clearance}")

def calculate_ray_casting_clearance(costmap, x, y, robot_radius, resolution):
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

def save_clearance_data_to_csv():
    with open('clearance_data.csv', 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['Timestep', 'Clearance'])
        for t, clearance in zip(indices, clearance_values):
            writer.writerow([t, clearance])

def plot_clearance_data(x, y):
    plt.figure()
    plt.plot(x, y, 'bo-', label='Clearance')
    plt.xlabel('Data Points')
    plt.ylabel('Clearance (meters)')
    plt.title('Clearance Metric')
    plt.legend()
    plt.grid(True)
    plt.show()

def main(args=None):
    print("Starting clearance calculator node.")
    rclpy.init(args=args)
    global node
    node = rclpy.create_node('clearance_calculator')

    # Set the logging level to INFO
    rclpy.logging.set_logger_level(node.get_logger().name, rclpy.logging.LoggingSeverity.INFO)

    # Global variables to store costmap data, global path, and global resolution
    global costmap_data
    global global_path
    global global_resolution
    global clearance_timer
    global clearance_values
    global timestamps
    global indices

    clearance_timer = None
    clearance_values = []
    timestamps = []
    indices = []
    global_path = [] 

    # Add a print statement to check if the node is running
    print("Clearance calculator node is running.")

    # Create subscribers
    costmap_subscriber = node.create_subscription(OccupancyGrid, '/global_costmap/costmap', costmap_callback, 10)
    global_path_subscriber = node.create_subscription(Path, '/plan', global_path_callback, 10)

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        # Save clearance data to a CSV file
        save_clearance_data_to_csv()

        # Plot the clearance data
        plot_clearance_data(indices, clearance_values)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
