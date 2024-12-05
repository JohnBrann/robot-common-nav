import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from wheel_nav_msgs.srv import Waypoint

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

class WaypointService(Node):
    def __init__(self):
        super().__init__('waypoint_service')
        self.navigator = BasicNavigator()
        self.srv = self.create_service(Waypoint, 'waypoint', self.waypoint_callback)

    def waypoint_callback(self, request, response):
        try:
            # Create PoseStamped from service request
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = request.reference_frame_id
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.pose = request.pose

            # Send the goal pose to the navigator
            self.navigator.goToPose(goal_pose)

            # Wait for the task to complete
            while not self.navigator.isTaskComplete(): # IS THIS BLOCKING????
                self.navigator.goToPose(goal_pose)

            # Wait for the task to complete
            # while not self.navigator.isTaskComplete():
            #     rclpy.spin_once(self, timeout_sec=0.1) 

            # Check the result of navigation
            result = self.navigator.getResult()

            if result == TaskResult.SUCCEEDED:
                self.get_logger().info(f"Successfully reached waypoint")
                response.result = "SUCCESS"
            elif result == TaskResult.CANCELED:
                self.get_logger().warning(f"Navigation canceled at waypoint")
                response.result = "CANCELLED"
            elif result == TaskResult.FAILED:
                self.get_logger().error(f"Failed to reach waypoint")
                response.result = "FAILED"
            else:
                self.get_logger().info("Unknown navigation result.")
                response.result = "UNKNOWN RESULT"

        except Exception as e:
            self.get_logger().error(f"Error in waypoint_callback: {str(e)}")
            response.result = "Error during navigation"

        return response

def main(args=None):
    rclpy.init(args=args)
    node = WaypointService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
