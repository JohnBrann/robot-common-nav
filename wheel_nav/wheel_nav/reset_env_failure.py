import rclpy
import py_trees
import py_trees_ros

from wheel_nav_msgs.msg import StepData

class ResetEnvFailure(py_trees.behaviour.Behaviour):
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

    def setup(self):
        """
        Setup any delayed initialization needed for the behavior.
        """
        # self.node.get_logger().info("Setting up ResetEnvState behavior...")

    def initialise(self):
        """
        Called the first time the behavior is ticked or anytime the status is not RUNNING thereafter.
        """
        self.node.get_logger().info("Resetting environment state after failure...")

    def update(self):
        """
        Reset the environment, reward, and other state data, respawn robot

        Returns:
            py_trees.common.Status: SUCCESS if the reset is successful, FAILURE otherwise.
        """
        try:
            # Publish reset state data
            reset_state = StepData()
            # reset_state.linear_velocity = 0.0
            # reset_state.angular_velocity = 0.0
            reset_state.success = False
            reset_state.terminated = False
            reset_state.reward = 0.0

            # todo: respawn robot!!!!
            self.publisher.publish(reset_state)

            self.node.reset_step_count()
            self.node.add_to_episode_count()
            current_episode = self.node.get_current_episode()
            episode_reward = self.node.get_current_episode_reward()
            self.node.get_logger().info(f"Episode {current_episode} Reward: {episode_reward}")
            
            self.node.get_logger().info("Environment reset successfully")
            return py_trees.common.Status.SUCCESS

        except Exception as e:
            self.node.get_logger().error(f"Failed to reset environment: {e}")
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        """
        Clean up when the behavior switches to a non-running state.
        """
        self.node.get_logger().info(f"Episode ended from Failure...")
