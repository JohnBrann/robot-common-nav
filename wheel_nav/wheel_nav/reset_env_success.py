import rclpy
import py_trees
import py_trees_ros
import torch 

from wheel_nav_msgs.srv import GoalUpdate
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3

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
        self.goal_update_client = self.node.create_client(GoalUpdate, 'goal_update') 
        self.marker_update_client = self.node.create_client(SetEntityState, '/gazebo/set_entity_state')

    def setup(self):
        """
        Setup any delayed initialization needed for the behavior.
        """

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

            # Reset step count and episode count
            self.node.reset_step_count()
            self.node.add_to_episode_count()
            current_episode = self.node.get_current_episode()
            episode_reward = self.node.get_current_episode_reward()


            if current_episode > self.node.best_episode_reward:
                if self.node.dqn_state:
                    torch.save(self.node.policy_dqn.state_dict(), self.node.MODEL_FILE)
                elif self.node.ddpg_state:
                    torch.save(self.node.actor.state_dict(), self.node.MODEL_FILE)
                self.node.best_episode_reward = episode_reward
                
            self.node.get_logger().info(f"Episode {current_episode} Reward: {episode_reward}")

            self.node.episode_rewards.append(episode_reward)
            self.node.add_to_results(1)

            # Reset episode reward
            self.node.reset_episode_reward()

            # decay epsilon value
            if self.node.epsilon > 0.1:
                self.node.epsilon = self.node.epsilon * 0.985
                self.node.epsilon_list.append(self.node.epsilon)
            
    
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
                    new_x = future.result().x
                    new_y = future.result().y
                    self.node.get_logger().info(f"\n\t\t\t\t\t    Goal updated successfully. New Goal: ({new_x} ,{new_y})")
                else:
                    self.node.get_logger().error("Goal update failed")
                    return py_trees.common.Status.FAILURE
            else:
                self.node.get_logger().error("Service call failed")
                return py_trees.common.Status.FAILURE
            

            '''
            Update Waypoint Marker location (same as new goal)

            '''
            marker_request = SetEntityState.Request()
            marker_request.state = EntityState(
                name='waypoint_marker',
                pose=Pose(
                    position=Point(x=new_x, y=new_y, z=0.1),
                    orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                ),
                twist=Twist(
                    linear=Vector3(x=0.0, y=0.0, z=0.0),
                    angular=Vector3(x=0.0, y=0.0, z=0.0)
                ),
                reference_frame='world'
            )
             # Call the service
            marker_future = self.marker_update_client.call_async(marker_request)
            rclpy.spin_until_future_complete(self.node, marker_future)
        
            # if marker_future.result() is not None:
            #     self.node.get_logger().info(f'Success: {marker_future.result().success}')
            # else:
            #     self.node.get_logger().error('Failed to call service')

            return py_trees.common.Status.SUCCESS
        
        except Exception as e:
            self.node.get_logger().error(f"Failed to reset environment: {e}")
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        """
        Clean up when the behavior switches to a non-running state.
        """
