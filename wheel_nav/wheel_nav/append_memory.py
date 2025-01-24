import rclpy
import py_trees
import py_trees_ros

class AppendMemory(py_trees.behaviour.Behaviour):
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
        One-time initialization that might be required for this behavior.
        Here we can print or log messages if needed.
        """

    def initialise(self):
        """
        This is called the first time the behaviour is ticked and anytime the status is not RUNNING thereafter.
        """

    def update(self):
        """
        Appends state, action, etc. data to memory buffer
        
        Returns:
            py_trees.common.Status: SUCCESS if reward calculation is successful, FAILURE otherwise.
        """

        self.node.memory.append((
            self.node.state,
            self.node.action,
            self.node.new_state,
            self.node.reward,
            self.node.terminated
        ))

        # sample = self.node.memory.sample(1) 
        # self.node.get_logger().info(f"Sampled Transitions: {sample}")

        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        """
        This is called when the behaviour switches to a non-running state (SUCCESS, FAILURE, INVALID).
        """
