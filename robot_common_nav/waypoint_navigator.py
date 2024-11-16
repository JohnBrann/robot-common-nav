import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from navigation_msgs.srv import Waypoint
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


class WaypointNavigator(Node):
    def __init__(self):
        super().__init__('waypoint_navigator')
        self.navigator = BasicNavigator()
        self.srv = self.create_service(Waypoint, 'waypoint', self.waypoint_callback)
        self.current_goal = None  # To track the current navigation goal
        self.is_navigating = False  # Flag to track navigation state

    def waypoint_callback(self, request, response):
        self.get_logger().info(f"Incoming waypoint request: Pose=({request.pose.position.x}, {request.pose.position.y})")

        # Create and send a new goal
        self.current_goal = PoseStamped()
        self.current_goal.header.frame_id = "map"
        self.current_goal.pose.position.x = request.pose.position.x
        self.current_goal.pose.position.y = request.pose.position.y
        self.current_goal.pose.orientation.w = 1.0  # Default orientation

        self.get_logger().info(f"Sending goal for waypoint ({request.pose.position.x}, {request.pose.position.y})")
        self.navigator.goToPose(self.current_goal)
        self.is_navigating = True

        response.result = "Navigation started."
        return response

    def check_navigation_status(self):
        """Check the navigation status and log results."""
        if self.is_navigating and self.navigator.isTaskComplete():
            result = self.navigator.getResult()

            if result == TaskResult.SUCCEEDED:
                self.get_logger().info(f"Successfully reached waypoint: ({self.current_goal.pose.position.x}, {self.current_goal.pose.position.y})")
            elif result == TaskResult.CANCELED:
                self.get_logger().warning(f"Navigation canceled at waypoint: ({self.current_goal.pose.position.x}, {self.current_goal.pose.position.y})")
            elif result == TaskResult.FAILED:
                self.get_logger().error(f"Failed to reach waypoint: ({self.current_goal.pose.position.x}, {self.current_goal.pose.position.y})")
            else:
                self.get_logger().info("Unknown navigation result.")

            # Reset navigation state
            self.current_goal = None
            self.is_navigating = False


def main(args=None):
    rclpy.init(args=args)
    waypoint_navigator = WaypointNavigator()

    try:
        while rclpy.ok():
            rclpy.spin_once(waypoint_navigator, timeout_sec=0.1)

            # Periodically check the navigation status
            waypoint_navigator.check_navigation_status()
    except KeyboardInterrupt:
        waypoint_navigator.get_logger().info("Shutting down waypoint navigator.")
    finally:
        waypoint_navigator.navigator.cancelTask()  # Cancel any ongoing navigation
        waypoint_navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
