#! /usr/bin/env python3

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration

def get_user_defined_pose(goalpose):

    if goalpose == 1:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = 11.0
        pose.pose.position.y = -13.0
        pose.pose.orientation.w = 1.0
        return pose
    elif goalpose == 2:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = 2.0
        pose.pose.position.y = -5.0
        pose.pose.orientation.w = 1.0
        return pose
    elif goalpose == 3:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = -7.0
        pose.pose.position.y = -9.0
        pose.pose.orientation.w = 1.0
        return pose
    else:
        raise ValueError("Invalid goal pose !")

def main():
    rclpy.init()

    navigator = BasicNavigator()
    navigator.waitUntilNav2Active()

    while True:
        # Get the user-defined variable for the goal pose
        goal_pose_variable = int(input("Enter goal pose (1, 2, or 3), or enter 0 to exit: "))

        if goal_pose_variable == 0:
            break  # Exit the loop if the user enters 0

        # Get the user-defined goal pose based on the variable
        goal_pose = get_user_defined_pose(goal_pose_variable)

        navigator.goToPose(goal_pose)

        i = 0
        while not navigator.isTaskComplete():
            # Do something with the feedback
            i += 1
            feedback = navigator.getFeedback()
            if feedback and i % 5 == 0:
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    navigator.cancelTask()

                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
                    goal_pose = get_user_defined_pose(goal_pose_variable)
                    navigator.goToPose(goal_pose)

        # Do something depending on the return code
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded! Robot has arrived at the goal pose.')
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')

    print('Exiting the program.')
    exit(0)

if __name__ == '__main__':
    main()

