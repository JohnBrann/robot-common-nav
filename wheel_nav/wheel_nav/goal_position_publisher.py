import rclpy
from rclpy.node import Node
from wheel_nav_msgs.msg import GoalPosition 
from wheel_nav_msgs.msg import StepData 
import random

class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal_publisher')

        # Publisher for new goal positions
        self.publisher_ = self.create_publisher(GoalPosition, 'goal_position', 10)

        # Subscribe to StepData topic
        self.subscription = self.create_subscription(
            StepData,
            'step_data',
            self.step_data_callback,
            10
        )

        # Define the range for generating random goals
        self.goal_range = (-1.8, 1.8)

    def generate_random_goal(self):
        """Generate a random goal position within the specified range."""
        x = random.uniform(self.goal_range[0], self.goal_range[1])
        y = random.uniform(self.goal_range[0], self.goal_range[1])
        return [x, y]

    def step_data_callback(self, msg):
        """
        Callback to process StepData messages.

        Args:
            msg (StepData): The message containing robot state information.
        """
        if msg.success or msg.terminated:  # Trigger new goal on success or termination
            self.get_logger().info(
                f"Event detected: {'Success' if msg.success else 'Termination'}"
            )
            self.publish_new_goal()

    def publish_new_goal(self):
        """Generate and publish a new random goal position."""
        goal = self.generate_random_goal()
        msg = GoalPosition()
        msg.x = goal[0]
        msg.y = goal[1]
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published new goal position: ({msg.x}, {msg.y})')


def main(args=None):
    rclpy.init(args=args)
    node = GoalPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
