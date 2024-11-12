import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        # Initialize the Navigator
        self.navigator = BasicNavigator()
        
    def move_to_waypoint(self, x, y):
        # Create a PoseStamped message with the target waypoint
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.w = 1.0
        
        # Send the goal to the navigator
        self.navigator.goToPose(goal)
        
        # Wait until the robot reaches the goal
        while not self.navigator.isTaskComplete():
            self.get_logger().info('Navigating...')
            rclpy.spin_once(self)
            print(f'here')

        
        print(f'finished')
        nav_result = self.navigator.getResult()
        if nav_result == 'SUCCEEDED':
            self.get_logger().info(f"Successfully reached waypoint: ({x}, {y})")
        else:
            self.get_logger().error(f"Failed to reach waypoint: ({x}, {y})")

def main(args=None):
    rclpy.init(args=args)
    waypoint_navigator = WaypointNavigator()
    
    # Waypoints (x, y coordinates)
    waypoints = [(0.1, 3.37), (2.5, 3.3), (1.0, 0.21)]
    
    for wp in waypoints:
        waypoint_navigator.move_to_waypoint(*wp)
    
    # Shutdown
    rclpy.shutdown()

if __name__ == '__main__':
    main()