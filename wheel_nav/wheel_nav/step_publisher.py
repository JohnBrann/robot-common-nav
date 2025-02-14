from wheel_nav import reward as rw
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from wheel_nav_msgs.msg import StepData 
from wheel_nav_msgs.msg import GoalPosition

from tf_transformations import euler_from_quaternion  # For quaternion to Euler conversion

import rclpy
from rclpy.node import Node
import numpy as np
import math

"""
Node: StepPublisher

Description:
    The StepPublisher node is responsible for publishing robot state information to the `step_data` topic.
    The node integrates data from odometry, LiDAR, and goal position to compute 
    essential metrics such as the distance and angle to the goal, obstacle proximity, and a calculated reward 
    based on the robot's performance.

Published Topic:
    - Topic: `step_data` (type: wheel_nav_msgs.msg.StepData)
        This topic publishes a `StepData` message containing the following fields:
        - `float32 distance_to_goal`: The Euclidean distance from the robot to the goal position.
        - `float32 angle_to_goal`: The angular difference between the robot's orientation and the goal.
        - `float32[] scan_data`: Selected LiDAR scan points representing the environment.
        - `float32 min_obstacle_distance`: The minimum distance to any obstacle detected by the LiDAR.
        - `float32 linear_velocity`: The current linear velocity of the robot.
        - `float32 angular_velocity`: The current angular velocity of the robot.
        - `bool success`: Indicates if the goal has been reached (distance < 0.25 meters).
        - `bool terminated`: Indicates if the episode was terminated due to collision or other criteria.
        - `float32 reward`: A reward signal calculated using the robot's performance metrics.
        - `float32 init_goal_distance`: The initial distance to the goal at the start of the episode.

Features:
    - Computes the Euclidean distance and angular difference to the goal.
    - Selects a subset of LiDAR scan points for compact representation and calculates the minimum obstacle distance.
    - Evaluates success when the robot is within goal distance
    - Triggers episode termination if the robot is too close to an obstacle
    - Resets episode data upon success or termination and updates the reward based on distance, angle, and velocities.

Usage:
    This node is designed to provide feedback for training reinforcement learning models in a simulation 
    environment.
"""


class StepPublisher(Node):
    def __init__(self):
        super().__init__('step_publisher')

        self.odom_initialized = False
        self.scan_initialized = False

        # Initialize state variables
        self.current_goal_pos = np.array([-1.0, 1.0])  
        self.current_pos = np.array([-1.0, 0.0])
        self.current_yaw = 0.0

        self.init_goal_distance = 1.0  # Initial distance to the goal
        self.episode_active = True  # Indicates if an episode is active
        self.linear_x = 0.0
        self.angular_z = 0.0
        self.scan_data = []
        self.min_obstacle_distance = 100.0
        self.reward = 0.0

        self.success = False
        self.terminated = False

        # Subscribers
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_subscriber = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.goal_subscriber = self.create_subscription(GoalPosition, 'goal_position', self.goal_callback, 10)

        # Publisher
        self.publisher = self.create_publisher(StepData, 'step_data', 10)

        # Timer for publishing data
        self.timer = self.create_timer(0.1, self.publish_data)

    def publish_data(self):
        if not self.odom_initialized or not self.scan_initialized:
            self.get_logger().warn('Publishing before odometry and scan data received')
            return

        distance_to_goal = self.calc_distance_to_goal()
        angle_to_goal = self.calc_angle_to_goal(self.current_yaw)

        # If the episode is inactive, reset for a new episode
        if not self.episode_active:
            self.init_goal_distance = distance_to_goal
            self.episode_active = True
            self.get_logger().info(f'New episode started. Initial goal distance: {self.init_goal_distance:.2f}')

        # Calculate reward
        self.reward = rw.calc_reward(
            distance_to_goal, self.init_goal_distance, angle_to_goal,
            self.min_obstacle_distance, self.success, self.terminated,
            self.linear_x, self.angular_z
        )

        # Prepare and publish the StepData message
        msg = StepData()
        msg.distance_to_goal = distance_to_goal
        msg.angle_to_goal = angle_to_goal
        msg.scan_data = self.scan_data
        msg.min_obstacle_distance = self.min_obstacle_distance
        msg.linear_velocity = self.linear_x
        msg.angular_velocity = self.angular_z
        msg.success = bool(self.success)
        msg.terminated = self.terminated
        msg.reward = self.reward
        msg.init_goal_distance = self.init_goal_distance # temp data

        self.publisher.publish(msg)
        self.get_logger().info(f'Published: {msg}')

        # If success or termination is detected, mark the episode as inactive
        if self.success or self.terminated:
            self.episode_active = False

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        self.current_pos = np.array([position.x, position.y])

        self.linear_x = msg.twist.twist.linear.x
        self.angular_z = msg.twist.twist.angular.z

        orientation = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([
            orientation.x, orientation.y, orientation.z, orientation.w
        ])
        self.current_yaw = yaw

        self.odom_initialized = True

    def lidar_callback(self, msg):
        total_ranges = len(msg.ranges)
        num_of_laser = 6
        step = total_ranges // num_of_laser
        selected_ranges = [msg.ranges[i] for i in range(0, total_ranges, step)][:num_of_laser]
        
        # Calculate the minimum distance to obstacles
        self.min_obstacle_distance = min(selected_ranges, default=10)
        
        # Terminate if too close to an obstacle
        self.terminated = self.min_obstacle_distance < 0.2
        self.scan_data = selected_ranges
        self.scan_initialized = True

    def goal_callback(self, msg):
        # Update the goal position but only reset the episode if it's inactive
        if not self.episode_active:
            self.current_goal_pos = np.array([msg.x, msg.y])
            self.get_logger().info(f'Goal updated to: {self.current_goal_pos}')
        else:
            self.get_logger().info(f'Ignored goal update during active episode')

    def calc_distance_to_goal(self):
        distance = np.sqrt(np.sum((self.current_pos - self.current_goal_pos) ** 2))
        self.success = distance < 0.25
        return distance

    def calc_angle_to_goal(self, yaw):
        goal_x, goal_y = self.current_goal_pos
        current_x, current_y = self.current_pos
        angle_to_goal = math.atan2(goal_y - current_y, goal_x - current_x)
        angle_diff = math.atan2(math.sin(angle_to_goal - yaw), math.cos(angle_to_goal - yaw))
        return abs(angle_diff)

def main(args=None):
    rclpy.init(args=args)
    step_publisher = StepPublisher()
    rclpy.spin(step_publisher)
    rclpy.shutdown()

if __name__ == '__main__':
    main()