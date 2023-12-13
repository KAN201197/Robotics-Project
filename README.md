# ME5400A Robotics Project

This project aims to design and develop an Autonomous Mobile Robot for food delivery using mecanum wheel robot. The objective of this project is for the robot to navigate into three different positions by calling or giving the coordinates of the exact position while avoiding every dynamic obstacle. This project is still going to develop the robot arm which is attached to the top of the mobile robot to become a mobile manipulator that can be used to manipulate the object (grip something or press something) as well as add another sensor such as GPS to be able to navigate more accurately in complex and dynamic environments outside of the building.The navigation stack on this project use ROS2 Nav2.

## List of Executable Files

Below are listed the executable files and folders in this repo:

1. Metric_Calculation

   Inside this folder has a list of executable files to measure a performance of the choosen global and local path planner by calculating the metric calculation such as path length, execution time, initialization time, etc.

2. gazebo_simulation

   Inside this folder has launch files inside launch folder to spawn the robot into the world in Gazebo. The URDF and Meshes folder used to define the robot description.

3. robot_nav2

   Inside this folder has launch files to initialize localization and navigation into the robot using Nav2 navigation stack. The param folder used to define every parameter related to AMCL and navigation used by the robot. Map folder used to save the map building by SLAM algorithm. The script folder has an executable file ("gotopose.py") used to give a command to the robot to move into three different position.

4. robot_slam

   Inside this folder has several folders used to do mapping with different SLAM algorithm. The available SLAM method is Carthographer, SLAM toolbox, and SLAM gmapping.
