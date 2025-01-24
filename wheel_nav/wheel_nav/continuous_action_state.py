import rclpy 
import py_trees
import py_trees_ros


class ContinuousActionState(py_trees.behaviour.Behaviour):
    def __init__(self, node, name, is_discrete):
        """
        Initialize the condition node with a reference to the ROS 2 node
        
        Args:
            name (str): Name of the behavior
            node (rclpy.node.Node): ROS 2 node to access parameters
        """
        super().__init__(name)
        self.node = node
        self.is_discrete = is_discrete
        

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
        Flag to see if the action to be selected is continuous
        
        Returns:
            py_trees.common.Status: SUCCESS if training is enabled, FAILURE otherwise
        """

        if self.is_discrete:
            return py_trees.common.Status.FAILURE
        else:
            # self.node.get_logger().info(f"Action Space is Continuous")
            return py_trees.common.Status.SUCCESS
        
    def terminate(self, new_status):
        """
        This is called when the behaviour switches to a non-running state.
            SUCCESS || FAILURE || INVALID
        """

