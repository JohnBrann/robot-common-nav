import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class LaserScanSubscriber(Node):
    def __init__(self):
        super().__init__('laser_scan_subscriber')
        
        
        # STATE DATA
        self.lidar_data = []
        self.distance_to_goal = None # from odon data, uclidean distance formula
        self.angle_to_goal = None
        self.linear_velocity = None
        self.angular_velocity = None
        self.closest_obstacle_distance = None

        self.scan_subscriber = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        self.current_goal_pos = np.array([1, 1])
        self.current_pos = np.array([0, 0])


    def lidar_callback(self, msg):
        self.get_logger().info('Received LaserScan message:')
        # self.get_logger().info(f'  Ranges: {msg.ranges}')


    def calc_distance_from_goal(self):
        # Use the euclidean distance formula to find the current distance from the goal 
        print(f'current_goal_pos: {self.current_goal_pos}')
        print(f'current_pos: {self.current_pos}')

        self.distance_to_goal = np.sqrt(np.sum((self.current_pos - self.current_goal_pos)**2))
        print(f'Distance from goal: {self.distance_to_goal}')

        return self.distance_to_goal
    
    def calc_angle_to_goal(self):

        return self.angle_to_goal


    def current_state_callback(self):
        state = state
        return state

def main(args=None):
    rclpy.init(args=args)
    laser_scan_subscriber = LaserScanSubscriber()
    rclpy.spin_once(laser_scan_subscriber)
    laser_scan_subscriber.calc_distance_from_goal()
    rclpy.shutdown()

if __name__ == '__main__':
    main()