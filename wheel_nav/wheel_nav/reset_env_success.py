import rclpy
import py_trees
import py_trees_ros
from wheel_nav_msgs.srv import GoalUpdate

import torch 

class ResetEnvSuccess(py_trees.behaviour.Behaviour):
    def __init__(self, node, name):
        """
        Initialize the reset behavior with a reference to the ROS 2 node.

        Args:
            name (str): Name of the behavior
            node (rclpy.node.Node): ROS 2 node to access parameters
        """
        super().__init__(name)
        self.node = node
        self.goal_update_client = self.node.create_client(GoalUpdate, 'goal_update')  # Service client

    def setup(self):
        """
        Setup any delayed initialization needed for the behavior.
        """
        # self.node.get_logger().info("Setting up ResetEnvState behavior...")

    def initialise(self):
        """
        Called the first time the behavior is ticked or anytime the status is not RUNNING thereafter.
        """
        self.node.get_logger().info("Resetting environment state after success...")

    def update(self):
        """
        Reset the environment, reward, and other state data, and update the goal via the service.

        Returns:
            py_trees.common.Status: SUCCESS if the reset is successful, FAILURE otherwise.
        """
        try:
            self.node.success = False
            self.node.terminated = False
            self.node.reward = 0.0

            # Set parameters related to the episode state
            # self.node.set_parameters([
            #     rclpy.parameter.Parameter('success', rclpy.Parameter.Type.BOOL, False),
            #     rclpy.parameter.Parameter('terminated', rclpy.Parameter.Type.BOOL, False),
            #     rclpy.parameter.Parameter('reward', rclpy.Parameter.Type.DOUBLE, 0.0),
            # ])

            # Reset step count and episode count
            self.node.reset_step_count()
            self.node.add_to_episode_count()
            current_episode = self.node.get_current_episode()
            episode_reward = self.node.get_current_episode_reward()


            if current_episode > self.node.best_episode_reward:
                torch.save(self.node.policy_dqn.state_dict(), self.node.MODEL_FILE)
                self.node.best_episode_reward = episode_reward
                
            self.node.get_logger().info(f"Episode {current_episode} Reward: {episode_reward}")

            self.node.episode_rewards.append(episode_reward)

            # Reset episode reward
            self.node.reset_episode_reward()
            
            

            # Make the service call to update the goal
            if not self.goal_update_client.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().error('Goal Update service not available')
                return py_trees.common.Status.FAILURE

            # Create request object
            request = GoalUpdate.Request()
            # request.x = 1.0  # Example coordinates, replace with actual logic
            # request.y = 0.0  # Example coordinates, replace with actual logic

            # Call the service
            future = self.goal_update_client.call_async(request)

            # Wait for the service response
            rclpy.spin_until_future_complete(self.node, future)
            if future.result() is not None:
                success = future.result().success
                if success:
                    self.node.get_logger().info("Goal updated successfully")
                else:
                    self.node.get_logger().error("Goal update failed")
                    return py_trees.common.Status.FAILURE
            else:
                self.node.get_logger().error("Service call failed")
                return py_trees.common.Status.FAILURE

            self.node.get_logger().info("Environment reset successfully")
            return py_trees.common.Status.SUCCESS

        except Exception as e:
            self.node.get_logger().error(f"Failed to reset environment: {e}")
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        """
        Clean up when the behavior switches to a non-running state.
        """
        # self.node.get_logger().info(f"ResetEnvState terminated with status: {new_status}")
