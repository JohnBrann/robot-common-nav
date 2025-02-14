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
        
    def initialise(self):
        """
        This is called the first time the behaviour is ticked and anytime the status is not RUNNING thereafter.
        """
        
    def update(self):
        """
        Most DRL algorithms have delayed optimization, meaning optimzation does not happen after every step
        This behavior acts as a flag that allow the model to optimize after n steps
        Returns:
            py_trees.common.Status: SUCCESS if training is enabled, FAILURE otherwise
        """
        if self.node.step_count % 1 == 0:
            # self.node.get_logger().info(f"ALLLLLLLOOOOOOOOOOOOWWWWWWWWWWWWWWWWWw")
            return py_trees.common.Status.FAILURE

        if self.node.allow_optimization:
            # self.node.get_logger().info(f"ALLLLLLLOOOOOOOOOOOOWWWWWWWWWWWWWWWWWw")
            return py_trees.common.Status.FAILURE
        else:
            return py_trees.common.Status.SUCCESS
        
    def terminate(self, new_status):
        """
        This is called when the behaviour switches to a non-running state.
            SUCCESS || FAILURE || INVALID
        """
        self.node.allow_optimization = False
        

