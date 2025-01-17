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

        # Initialize the reward variable
        self.reward = None

        # Subscribe to the StepData topic
        # self.subscription = self.node.create_subscription(
        #     StepData,
        #     'step_data',
        #     self.listener_callback,
        #     10
        # )
        # self.subscription  # prevent unused variable warning # do we need? 

    def setup(self):
        """
        One-time initialization that might be required for this behavior.
        Here we can print or log messages if needed.
        """
        # self.node.get_logger().info("Reward calculation setup completed.")

    def initialise(self):
        """
        This is called the first time the behaviour is ticked and anytime the status is not RUNNING thereafter.
        """
        # self.node.get_logger().info("Calculating reward from current state...")

    def update(self):
        """
        Calculate the current reward given the current state of the robot.
        
        Returns:
            py_trees.common.Status: SUCCESS if reward calculation is successful, FAILURE otherwise.
        """

        # self.node.get_logger().info(f"{self.node.reward}................")

        self.reward = self.node.reward
        # If reward is not yet available, return RUNNING (waiting for data)
        if self.reward is None:
            # self.node.get_logger().info("Waiting for reward data...")
            return py_trees.common.Status.RUNNING
        
        # self.node.episode_rewards.append(self.reward)
        
        self.reward = round(self.reward, 2) 
        
        # Log and return the current reward
        self.node.get_logger().info(f"Current Step Reward: {self.reward}")

        self.node.add_reward_to_episode(self.reward) # Adds the current reward from this step to the cumulative epsisode reward
        self.node.add_to_step_count() # make this its own behavior?

        # Return SUCCESS if reward is available
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        """
        This is called when the behaviour switches to a non-running state (SUCCESS, FAILURE, INVALID).
        """
        # if new_status != py_trees.common.Status.RUNNING:
        #     self.node.get_logger().info(f"Terminating reward calculation with status {new_status}")

    # def listener_callback(self, msg):
    #     """
    #     Callback to handle incoming messages and extract reward data.
        
    #     Args:
    #         msg (StepData): The incoming message containing the reward.
    #     """
    #     # Extract reward from the message
    #     self.reward = msg.reward
        # self.node.get_logger().info(f"Received step reward: {self.reward}")
