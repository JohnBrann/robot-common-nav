import rclpy 
import py_trees
import py_trees_ros

class DDPGState(py_trees.behaviour.Behaviour):
    def __init__(self, node, name, ddpg_state):
        """
        Initialize the condition node with a reference to the ROS 2 node
        
        Args:
            name (str): Name of the behavior
            node (rclpy.node.Node): ROS 2 node to access parameters
        """
        super().__init__(name)
        self.node = node
        self.ddpg_state = ddpg_state
        
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
        Checks if algorithm being used is DQN
        **Currently, I am just seeing whether a a flag for the algorithm is true or false. In future development with multiple algorithms
            a different method might be cleaner
        
        Returns:
            py_trees.common.Status: SUCCESS if training is enabled, FAILURE otherwise
        """
        if self.ddpg_state:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE
        
    def terminate(self, new_status):
        """
        This is called when the behaviour switches to a non-running state.
            SUCCESS || FAILURE || INVALID
        """

