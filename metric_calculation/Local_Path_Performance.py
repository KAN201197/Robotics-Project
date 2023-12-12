#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav2_simple_commander.robot_navigator import BasicNavigator
from nav2_util import LocalPlannerHandler

class NavigationAndMetrics(Node):
    def __init__(self):
        super().__init__('navigation_and_metrics')
        self.navigator = BasicNavigatorNavigator()
        self.local_planner_handler = LocalPlannerHandler(self)
        self.local_planner = self.local_planner_handler.create_local_planner("dwb_local_planner/DWBLocalPlanner")
        self.local_planner.configure()
        self.get_logger().info("NavigationAndMetrics node initialized")

    def plan_and_evaluate(self):
        start_pose = PoseStamped()
        start_pose.header.frame_id = 'map'
        start_pose.pose.position.x = 0.0
        start_pose.pose.position.y = 0.0

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.pose.position.x = 5.0
        goal_pose.pose.position.y = 5.0

        # Plan a path to the goal
        path_result = self.navigator.get_path(start_pose, goal_pose)
        if not path_result:
            self.get_logger().error("Failed to plan a path.")
            return

        # Evaluate collision metric
        collision_metric = self.evaluate_collision_metric(path_result.path)

        # Evaluate deviation distance
        deviation_distance = self.evaluate_deviation_distance(path_result.path)

        self.get_logger().info(f"Collision Metric: {collision_metric}")
        self.get_logger().info(f"Deviation Distance: {deviation_distance}")

    def evaluate_collision_metric(self, path):
        total_collision_cost = 0
        costmap = self.local_planner.get_costmap()
        collision_checker = self.local_planner.get_collision_checker()
        
        for pose in path.poses:
            mx, my = costmap.worldToMapValidated(pose.pose.position.x, pose.pose.position.y)
            cost = collision_checker.pointCost(mx, my)
            total_collision_cost += cost
        
        return total_collision_cost / len(path.poses)

    def evaluate_deviation_distance(self, path):
        deviation_distance = 0
        robot_pose = self.local_planner.get_current_pose()

        for pose in path.poses:
            dx = robot_pose.pose.position.x - pose.pose.position.x
            dy = robot_pose.pose.position.y - pose.pose.position.y
            deviation_distance += (dx ** 2 + dy ** 2) ** 0.5
        
        return deviation_distance

def main(args=None):
    rclpy.init(args=args)
    nav_and_metrics = NavigationAndMetrics()

    try:
        nav_and_metrics.plan_and_evaluate()
    except KeyboardInterrupt:
        pass

    nav_and_metrics.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
