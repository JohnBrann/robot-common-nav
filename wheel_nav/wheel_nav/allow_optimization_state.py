import rclpy 
import py_trees
import py_trees_ros


class AllowOptimizationState(py_trees.behaviour.Behaviour):
    def __init__(self, node, name):
        """
        Initialize the condition node with a reference to the ROS 2 node
        
        Args:
            name (str): Name of the behavior
            node (rclpy.node.Node): ROS 2 node to access parameters
        """
        super().__init__(name)
        self.node = node

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
        We only want to optimze the model at the end of an episode
        Flag created for this 
        Returns:
            py_trees.common.Status: SUCCESS if training is enabled, FAILURE otherwise
        """


        if self.node.step_count % 10 == 0:
            self.node.get_logger().info(f"\n\n\nOPTMIZEZEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE\n\n\n")
            return py_trees.common.Status.FAILURE

        if self.node.allow_optimization:
            # self.node.get_logger().info(f"OPTMIZEZEEEEEEEEEEEEEEE")
            return py_trees.common.Status.FAILURE
        else:
            return py_trees.common.Status.SUCCESS
        
    def terminate(self, new_status):
        """
        This is called when the behaviour switches to a non-running state.
            SUCCESS || FAILURE || INVALID
        """
        self.node.allow_optimization = False
        # self.node.get_logger().info(f"Terminating TrainingModeState with status {new_status}")

