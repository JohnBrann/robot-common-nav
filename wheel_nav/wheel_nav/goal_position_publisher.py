import rclpy
from rclpy.node import Node
from wheel_nav_msgs.srv import GetGoalPosition 
import random

class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal_publisher')

        # Define the range for generating random goals
        self.goal_range = (-1.8, 1.8)

        # Create a service to get a goal position
        self.srv = self.create_service(GetGoalPosition, 'get_goal_position', self.get_goal_position_callback)

    def generate_random_goal(self):
        """Generate a random goal position within the specified range."""
        x = random.uniform(self.goal_range[0], self.goal_range[1])
        y = random.uniform(self.goal_range[0], self.goal_range[1])
        return [x, y]

    def get_goal_position_callback(self, request, response):
        """Service callback to provide a random goal position."""
        goal = self.generate_random_goal()
        response.goal.x = goal[0]
        response.goal.y = goal[1]
        self.get_logger().info(f'Serving new goal position: ({response.goal.x}, {response.goal.y})')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = GoalPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
