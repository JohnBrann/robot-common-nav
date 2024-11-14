import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import time

class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        self.navigator = BasicNavigator()

    def move_to_waypoint(self, x, y):
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.pose.position.x = x
        goal.pose.position.y = y
        # goal.pose.orientation.w = 1.0

        self.get_logger().info(f"Sending goal for waypoint ({x}, {y})")
        self.navigator.goToPose(goal)

        while True:
            # Get the current status of navigation
            if self.navigator.isTaskComplete():
                result = self.navigator.getResult()
                if result == TaskResult.SUCCEEDED:
                    self.get_logger().info(f"Successfully reached waypoint: ({x}, {y})")
                    return True
                elif result == TaskResult.CANCELED:
                    self.get_logger().warning(f"Navigation canceled at waypoint: ({x}, {y})")
                    return False
                elif result == TaskResult.FAILED:
                    self.get_logger().error(f"Failed to reach waypoint: ({x}, {y})")
                    return False

            rclpy.spin_once(self, timeout_sec=1.0)

    def follow_waypoints(self, waypoints):
        # Set initial pose of robot in Rviz
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = "map"

        # In future this should change to where the robot tries to find initial pose itself
        # The map was created from the same spawning location, so initial pose in map is (0, 0)
        initial_pose.pose.position.x = 0.0
        initial_pose.pose.position.y = 0.0
        initial_pose.pose.orientation.w = 1.0

        self.navigator.setInitialPose(initial_pose)
    
        self.navigator.waitUntilNav2Active()

        # Navigate through waypoints
        for i, (x, y) in enumerate(waypoints):
            self.get_logger().info(f"Starting navigation to waypoint {i+1}/{len(waypoints)}: ({x}, {y})")
            
            success = self.move_to_waypoint(x, y)
            
            if not success:
                self.get_logger().error(f"Failed to reach waypoint {i+1}, stopping navigation")
                return
            
            self.get_logger().info(f"Completed waypoint {i+1}/{len(waypoints)}")
            
            # Clear costmaps between waypoints
            # May be beneficial to clear a local costmap so robot can re-evaluate enviroment
            self.navigator.clearAllCostmaps()
            time.sleep(2.0) 

def main(args=None):
    rclpy.init(args=args)
    waypoint_navigator = WaypointNavigator()
    
    waypoints = [(0.1, 3.37), (2.5, 3.3), (1.0, 0.21)]

    waypoint_navigator.follow_waypoints(waypoints)

if __name__ == '__main__':
    main()