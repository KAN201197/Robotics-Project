# ME5400A Robotics Project

![image](https://github.com/KAN201197/Robotics-Project/assets/128454220/ac44fd33-36aa-4b1c-b07e-31837be8374f)

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

5. Pathplanning Matlab Simulink

   Inside this folder has several executable file to open Matlab Simulink to simulate path planning (global and local path planner) with ROS.

## Matlab Simulink Simulation

1. Simulink Diagram Process of Global and Local Path planner which get input from lidar and odometry data and send output to command velocity publisher in ROS topic.
   
   ![image](https://github.com/KAN201197/Robotics-Project/assets/128454220/d1eaf7c1-2bf4-4c4f-96ed-7e5f57bda7ad)

2. Simulation result in Gazebo with Matlab Simulink for global and local path planning.
   
   ![TrialAvoidObstacle2-ezgif com-crop](https://github.com/KAN201197/Robotics-Project/assets/128454220/dc242fe8-4038-4f1b-915f-96d4d66663f5)

## Nav2 Simulation and Performance Evaluation using Metric

1. Overall Metric Calculation for global path planner can be seen from image below

   ![image](https://github.com/KAN201197/Robotics-Project/assets/128454220/b368453b-0bbb-4070-983c-9224a21844c3)

2. Overall Metric Calculation for local path planner can be seen from image below

   ![image](https://github.com/KAN201197/Robotics-Project/assets/128454220/6d2a4cf3-1eea-4fb2-b36b-bb072b9bd8a0)

3. The Nav2 Simulation result using A* global path planner and DWB local path planner

   ![Abletoavoidmedium-bigobstacle7s-ezgif com-video-to-gif-converter](https://github.com/KAN201197/Robotics-Project/assets/128454220/bb0cccac-3e7e-42fd-bdc5-6ea9cdd3b82d)

## Demo Result on Physical Robot

![DemoVideo-ezgif com-video-to-gif-converter (2)](https://github.com/KAN201197/Robotics-Project/assets/128454220/c774f87d-3d2e-45c8-ae17-9be0b931531e)






