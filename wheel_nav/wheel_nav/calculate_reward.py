import rclpy
import py_trees
import py_trees_ros

from wheel_nav_msgs.msg import StepData

class CalcReward(py_trees.behaviour.Behaviour):
    def __init__(self, node, name):
        """
        Initialize the condition node with a reference to the ROS 2 node
        
        Args:
            name (str): Name of the behavior
            node (rclpy.node.Node): ROS 2 node to access parameters
        """
        super().__init__(name)
        self.node = node

        # Initialize reward variable
        self.reward = None

    def setup(self):
        """
        One-time initialization that might be required for this behavior.
        Here we can print or log messages if needed.
        """

    def initialise(self):
        """
        This is called the first time the behaviour is ticked and anytime the status is not RUNNING thereafter.
        """

    def update(self):
        """
        Get the current reward that was calculated for the current state and add to current episode
        cumulative reward
        
        Returns:
            py_trees.common.Status: SUCCESS if reward calculation is successful, FAILURE otherwise.
        """

        self.reward = self.node.reward
        # If reward is not yet available, return RUNNING (waiting for data)
        if self.reward is None:
            # self.node.get_logger().info("Waiting for reward data...")
            return py_trees.common.Status.RUNNING
        
        # self.reward = round(self.reward, 2) 
        
        # Log and return the current reward
        self.node.get_logger().info(f"Current Step Reward: {self.reward}")

        self.node.add_reward_to_episode(self.reward) # Adds the current reward from this step to the cumulative epsisode reward
        self.node.add_to_step_count() 

        # Return SUCCESS if reward is available
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        """
        This is called when the behaviour switches to a non-running state (SUCCESS, FAILURE, INVALID).
        """
    
