from wheel_nav import reward as rw
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from wheel_nav_msgs.msg import StepData
from wheel_nav_msgs.msg import GoalPosition
from wheel_nav_msgs.srv import GetGoalPosition  # Service definition

from tf_transformations import euler_from_quaternion  # For quaternion to Euler conversion

import rclpy
from rclpy.node import Node
import numpy as np
import math


class StepPublisher(Node):
    def __init__(self):
        super().__init__('step_publisher')

        self.odom_initialized = False
        self.scan_initialized = False

        self.current_goal_pos = np.array([0.0, 0.0])  # Whenever there is a success or failure, a new one should be generated
        self.current_pos = np.array([-1, 0])
        self.current_yaw = 0

        self.linear_x = 0
        self.angular_z = 0

        self.scan_data = []
        self.min_obstacle_distance = 100

        self.success = False
        self.terminated = False

        # Subscribers
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_subscriber = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        # Service Client to get the goal position
        self.goal_client = self.create_client(GetGoalPosition, 'get_goal_position')
        
        # Wait for the service to be available
        while not self.goal_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        # Publisher
        self.publisher = self.create_publisher(StepData, 'step_data', 10)

        # Timer for publishing data
        self.timer = self.create_timer(0.1, self.publish_data)  # Publish every 0.1 seconds

    def publish_data(self):
        if not self.odom_initialized or not self.scan_initialized:
            self.get_logger().warn('Publishing before odometry and scan data received')
            return

        distance_to_goal = self.calc_distance_to_goal()
        angle_to_goal = self.calc_angle_to_goal(self.current_yaw)

        # Calculate reward
        reward = rw.calc_reward(
            distance_to_goal, 2.82, angle_to_goal, self.min_obstacle_distance,
            self.success, self.terminated, self.linear_x, self.angular_z
        )

        # Prepare the StepData message
        msg = StepData()
        msg.distance_to_goal = distance_to_goal
        msg.angle_to_goal = angle_to_goal
        msg.scan_data = self.scan_data
        msg.min_obstacle_distance = self.min_obstacle_distance
        msg.linear_velocity = self.linear_x
        msg.angular_velocity = self.angular_z
        msg.success = self.success
        msg.terminated = self.terminated
        msg.reward = reward

        # Publish the message
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: {msg}')

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
        num_of_laser = 12
        step = total_ranges // num_of_laser
        selected_ranges = [msg.ranges[i] for i in range(0, total_ranges, step)][:num_of_laser]
        # Calculates the minimum distance to an obstacle from lidar data
        self.min_obstacle_distance = min(selected_ranges, default=10)
        # If robot is too close to an obstacle, result in termination
        if self.min_obstacle_distance < 0.2:
            self.terminated = True
        else:
            self.terminated = False
        self.scan_data = selected_ranges
        self.scan_initialized = True

    def get_goal_position(self):
        """Call the service to get a new goal position."""
        # Create a request for the service
        request = GetGoalPosition.Request()

        # Make the service call and get the response
        future = self.goal_client.call_async(request)
        future.add_done_callback(self.goal_position_callback)

    def goal_position_callback(self, future):
        """Callback to handle the response from the service."""
        try:
            response = future.result()
            self.current_goal_pos = np.array([response.goal.x, response.goal.y])
            self.get_logger().info(f'Received new goal position: ({self.current_goal_pos[0]}, {self.current_goal_pos[1]})')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')

    def calc_distance_to_goal(self):
        distance = np.sqrt(np.sum((self.current_pos - self.current_goal_pos) ** 2))
        if distance < 0.1:
            self.success = True
        else:
            self.success = False
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
    
    # Get the initial goal position
    step_publisher.get_goal_position()
    
    rclpy.spin(step_publisher)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
