import rclpy
import py_trees
import py_trees_ros

from wheel_nav_msgs.msg import StepData
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SetModelState, GetModelState
from gazebo_msgs.msg import ModelState


class ResetEnvState(py_trees.behaviour.Behaviour):
    def __init__(self, node, name):
        """
        Initialize the reset behavior with a reference to the ROS 2 node.

        Args:
            name (str): Name of the behavior
            node (rclpy.node.Node): ROS 2 node to access parameters
            is_training (bool): Indicates if the agent is in training mode
        """
        super().__init__(name)
        self.node = node
        self.publisher = self.node.create_publisher(StepData, 'step_data', 10)

        # Gazebo reset services
        self.set_model_state_client = self.node.create_client(SetModelState, '/gazebo/set_model_state')
        self.get_model_state_client = self.node.create_client(GetModelState, '/gazebo/get_model_state')

    def setup(self):
        """
        Setup any delayed initialization needed for the behavior.
        """
        self.node.get_logger().info("Setting up ResetEnvState behavior...")
        self.set_model_state_client.wait_for_service()
        self.get_model_state_client.wait_for_service()

    def initialise(self):
        """
        Called the first time the behavior is ticked or anytime the status is not RUNNING thereafter.
        """
        self.node.get_logger().info("Resetting environment state...")

    def update(self):
        """
        Reset the environment, including robot pose, reward, and other state data.

        Returns:
            py_trees.common.Status: SUCCESS if the reset is successful, FAILURE otherwise.
        """
        try:
            # Reset robot position in Gazebo
            self.reset_robot_position()

            # Publish reset state data
            reset_state = StepData()
            reset_state.distance_to_goal = self.calculate_distance_to_goal()
            reset_state.angle_to_goal = self.calculate_angle_to_goal()
            reset_state.scan_data = []  # Assume sensors will update this dynamically
            reset_state.min_obstacle_distance = float('inf')  # Placeholder
            reset_state.linear_velocity = 0.0
            reset_state.angular_velocity = 0.0
            reset_state.success = False
            reset_state.terminated = False
            reset_state.reward = 0.0
            self.publisher.publish(reset_state)

            self.feedback_message = "Environment reset successfully"
            self.node.get_logger().info("Environment reset successfully")
            return py_trees.common.Status.SUCCESS

        except Exception as e:
            self.feedback_message = f"Failed to reset environment: {e}"
            self.node.get_logger().error(f"Failed to reset environment: {e}")
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        """
        Clean up when the behavior switches to a non-running state.
        """
        self.node.get_logger().info(f"ResetEnvState terminated with status: {new_status}")

    def reset_robot_position(self):
        """
        Reset the robot's position in Gazebo using the SetModelState service.
        """
        model_state = ModelState()
        model_state.model_name = 'turtlebot3'  # Replace with your robot's model name
        model_state.pose.position.x = 0.0
        model_state.pose.position.y = 0.0
        model_state.pose.position.z = 0.0
        model_state.pose.orientation.x = 0.0
        model_state.pose.orientation.y = 0.0
        model_state.pose.orientation.z = 0.0
        model_state.pose.orientation.w = 1.0
        model_state.twist.linear.x = 0.0
        model_state.twist.linear.y = 0.0
        model_state.twist.linear.z = 0.0
        model_state.twist.angular.x = 0.0
        model_state.twist.angular.y = 0.0
        model_state.twist.angular.z = 0.0

        req = SetModelState.Request()
        req.model_state = model_state

        future = self.set_model_state_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)

        if future.result():
            self.node.get_logger().info("Robot position reset successfully.")
        else:
            raise RuntimeError("Failed to reset robot position in Gazebo.")

    def calculate_distance_to_goal(self):
        """
        Calculate the initial distance to the goal based on the robot's reset position.
        Replace with your own logic if necessary.
        """
        return 5.0  # Example value, replace with your own calculation

    def calculate_angle_to_goal(self):
        """
        Calculate the initial angle to the goal based on the robot's reset position.
        Replace with your own logic if necessary.
        """
        return 0.0  # Example value, replace with your own calculation
