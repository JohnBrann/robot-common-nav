import rclpy
from rclpy.node import Node
from wheel_nav_msgs.msg import GoalPosition
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState

class MarkerUpdater(Node):
    def __init__(self):
        super().__init__('waypoint_marker_node')
        
        # Subscribe to goal position updates
        self.goal_sub = self.create_subscription(
            GoalPosition,
            'goal_position',
            self.goal_callback,
            10
        )
        
        # Create service client for updating marker position
        self.set_entity_client = self.create_client(
            SetEntityState,
            '/gazebo/set_entity_state'
        )
        
        while not self.set_entity_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for Set Entity State service...')

    def goal_callback(self, msg):
        request = SetEntityState.Request()
        state = EntityState()
        state.name = 'waypoint_marker'  # Must match entity name in launch file
        state.pose.position.x = msg.x
        state.pose.position.y = msg.y
        state.pose.position.z = 0.01  # Keep slight elevation to remain visible
        
        request.state = state
        
        future = self.set_entity_client.call_async(request)
        future.add_done_callback(self.service_callback)

    def service_callback(self, future):
        try:
            response = future.result()
            if not response.success:
                self.get_logger().warn('Failed to update marker position')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    marker_updater = MarkerUpdater()
    rclpy.spin(marker_updater)
    rclpy.shutdown()

if __name__ == '__main__':
    main()