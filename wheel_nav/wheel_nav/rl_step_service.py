from wheel_nav_msgs.srv import Step
from wheel_nav import reward as rw
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from tf_transformations import euler_from_quaternion  # For quaternion to Euler conversion

import rclpy
from rclpy.node import Node
import numpy as np
import math 


class StepService(Node):
    def __init__(self):
        super().__init__('step_service')

        # Flag to track if first odometry message has been received
        self.odom_initialized = False
        self.scan_initialized = False

        self.current_goal_pos = np.array([-1.0, -1.0])
        self.current_pos = np.array([0, 0])
        self.current_yaw = 0

        self.linear_x = 0
        self.angular_z = 0
        
        self.scan_data = []
        # large value to compare with first lidar data
        self.min_obstacle_distance = 100

        self.success= False
        self.terminated = False
        
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_subscriber = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        self.srv = self.create_service(Step, 'step_service', self.step_callback)

        
    def step_callback(self, request, response):

        # Check if odometry has been initialized
        if not self.odom_initialized and not self.scan_initialized:
            self.get_logger().warn('Service called before odometry and scan data received')
            response.distance_to_goal = float('inf')
            response.angle_to_goal = float('inf')
            response.reward = 0.0
            return response

        response.distance_to_goal = self.calc_distance_to_goal()
        response.angle_to_goal = self.calc_angle_to_goal(self.current_yaw)
        response.scan_data = self.scan_data
        response.min_obstacle_distance = self.min_obstacle_distance
        response.linear_velocity = self.linear_x
        response.angular_velocity = self.angular_z
        response.terminated = self.terminated 
        response.success= self.success

        response.reward = rw.calc_reward(response.distance_to_goal, 2.82, response.angle_to_goal, self.min_obstacle_distance, 
                                         self.success, self.terminated, self.linear_x, self.angular_z)
        
        return response
    

    def odom_callback(self, msg):
        # Current pos
        position = msg.pose.pose.position
        current_x = position.x
        current_y = position.y
        self.current_pos = np.array([current_x, current_y])
        
        self.linear_x = msg.twist.twist.linear.x
        self.angular_z = msg.twist.twist.angular.z

        # Current yaw (orientation)
        orientation = msg.pose.pose.orientation

        _, _, yaw = euler_from_quaternion([
            orientation.x, orientation.y, orientation.z, orientation.w
        ])
        self.current_yaw = yaw

        self.odom_initialized = True

    
    def lidar_callback(self, msg):
        # self.get_logger().info(f'  Ranges: {len(msg.ranges)}')
        total_ranges = len(msg.ranges)
        num_of_laser = 4
        
        # Step size to pick evenly distributed points
        step = total_ranges // num_of_laser
        
        # Extract points evenly distributed
        selected_ranges = [msg.ranges[i] for i in range(0, total_ranges, step)]
        
        # If the number of selected points is greater than desired, truncate the list
        selected_ranges = selected_ranges[:num_of_laser]

        self.min_obstacle_distance = 10 # reset value for each scan
        for i in range(len(selected_ranges)):
            if self.min_obstacle_distance > selected_ranges[i]:
                self.min_obstacle_distance = selected_ranges[i]
        
        self.scan_data = selected_ranges

    
    def calc_distance_to_goal(self):
        # Use the euclidean distance formula to find the current distance from the goal         
        self.distance_to_goal = np.sqrt(np.sum((self.current_pos - self.current_goal_pos)**2))
        if self.distance_to_goal < 0.1:
            self.success= True

        return self.distance_to_goal
    
    
    def calc_angle_to_goal(self, yaw):
        goal_x = self.current_goal_pos[0]
        goal_y = self.current_goal_pos[1]
        
        current_x = self.current_pos[0]
        current_y = self.current_pos[1]

       # Angle to goal from robot's position
        angle_to_goal = math.atan2(goal_y - current_y, goal_x - current_x)

        # Angle difference (normalize to [-pi, pi])
        self.angle_to_goal = abs(math.atan2(math.sin(angle_to_goal - yaw), math.cos(angle_to_goal - yaw)))
        self.get_logger().info(f'Angle to goal: {math.degrees(self.angle_to_goal):.2f}°')
        return self.angle_to_goal

def main(args=None):
    rclpy.init(args=args)
    minimal_service = StepService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()