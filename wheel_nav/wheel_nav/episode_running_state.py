import rclpy 
import py_trees
import py_trees_ros

from wheel_nav_msgs.msg import StepData


class EpisodeRunningState(py_trees.behaviour.Behaviour):
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
        self.episode_running = None
        

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
        Check the episode is still in progress
        This is kinda just a place holder, to allow for the tree to continue running
        We first check if episode is a success or terminated on previous behaviors, if those fail
        Then we jsut want to continue the tree, otherwise if success on oen of those, things happen then they continure with the tree
        
        Returns:
            py_trees.common.Status: SUCCESS if training is enabled, FAILURE otherwise
        """

        self.episode_running = True

        self.node.allow_optimization = False

        if self.episode_running == True:
            self.node.get_logger().info(f"Episode Running")
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        """
        This is called when the behaviour switches to a non-running state.
            SUCCESS || FAILURE || INVALID
        """
        # self.node.get_logger().info(f"Terminating TrainingModeState with status {new_status}")