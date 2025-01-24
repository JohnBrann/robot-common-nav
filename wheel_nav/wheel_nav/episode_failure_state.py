import rclpy 
import py_trees
import py_trees_ros

from wheel_nav_msgs.msg import StepData


class EpisodeFailureState(py_trees.behaviour.Behaviour):
    def __init__(self, node, name):
        """
        Initialize the condition node with a reference to the ROS 2 node
        
        Args:
            name (str): Name of the behavior
            node (rclpy.node.Node): ROS 2 node to access parameters
        """
        super().__init__(name)
        self.node = node

        # Initialize the reward variable
        self.episode_failure = None        

    def setup(self):

        """
          What to do here?
          Delayed one-time initialisation that would otherwise interfere
            with offline rendering of this behaviour in a tree to dot graph
            or validation of the behaviour's configuration.
        """

    def initialise(self):
        """
        This is called the first time the behaviour is ticked and anytime the status is not RUNNING thereafter.
        """

    def update(self):
        """
        Check if the episode was a failure
        
        Returns:
            py_trees.common.Status: SUCCESS if training is enabled, FAILURE otherwise
        """

        self.episode_failure = self.node.terminated

        # Truncation (other faillure)
        if self.node.episode_truncated():
            self.episode_failure = True
            self.node.add_reward_to_episode(-100) # This is for truncated reward (fix in future to be tied direcly with reward function?)
        
        if self.episode_failure:
            self.node.get_logger().info(f"Episode was a FAILURE")
            self.node.allow_optimization = True
            return py_trees.common.Status.SUCCESS
        else:
            # self.node.get_logger().info(f"Agent is Testing")
            return py_trees.common.Status.FAILURE
        
    def terminate(self, new_status):
        """
        This is called when the behaviour switches to a non-running state.
            SUCCESS || FAILURE || INVALID
        """