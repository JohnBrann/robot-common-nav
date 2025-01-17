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
          What to do here?
          Delayed one-time initialisation that would otherwise interfere
            with offline rendering of this behaviour in a tree to dot graph
            or validation of the behaviour's configuration.
        """

        # self.node.get_logger().info(f"Setting up TrainingModeState with is_training = {self.is_training}")
        

    def initialise(self):
        """
        This is called the first time the behaviour is ticked and anytime the status is not RUNNING thereafter.
        """
        # self.node.get_logger().info(f"Determining Training Mode... is_training?")
        

    def update(self):
        """
        Check the episode was a failure, we will also check for a truncation but a termiantion is fien for now
        
        Returns:
            py_trees.common.Status: SUCCESS if training is enabled, FAILURE otherwise
        """

        # if self.node.episode_truncated():
        #     self.episode_failure = True
        self.episode_failure = self.node.terminated
        self.node.allow_optimization = True

        if self.node.episode_truncated():
            self.episode_failure = True
            self.node.add_reward_to_episode(-100)
        
        if self.episode_failure:
            self.node.get_logger().info(f"Episode was a FAILURE")
            return py_trees.common.Status.SUCCESS
        else:
            # self.node.get_logger().info(f"Agent is Testing")
            return py_trees.common.Status.FAILURE
        
    def terminate(self, new_status):
        """
        This is called when the behaviour switches to a non-running state.
            SUCCESS || FAILURE || INVALID
        """
        # self.node.get_logger().info(f"Terminating TrainingModeState with status {new_status}")

    # def listener_callback(self, msg):
    #     """
    #     Callback to handle incoming messages and extract episode data.
        
    #     Args:
    #         msg (StepData): The incoming message containing the reward.
    #     """
    #     # Episode success or failure
    #     self.episode_failure = msg.terminated # truncated? get step_count
    #     # self.node.get_logger().info(f"Received reward: {self.reward}")