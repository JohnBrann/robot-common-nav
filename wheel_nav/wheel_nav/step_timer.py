import rclpy
import py_trees
import py_trees_ros

from wheel_nav_msgs.msg import StepData

class StepTimer(py_trees.behaviour.Behaviour):
    def __init__(self, node, name):
        """
        Initialize the condition node with a reference to the ROS 2 node
        
        Args:
            name (str): Name of the behavior
            node (rclpy.node.Node): ROS 2 node to access parameters
        """
        super().__init__(name)
        self.node = node

        # Initialize the timer
        self.time_limit_reached = False

        self.timer = self.node.create_timer(0.2, self.timer_callback) # 1.0 is the period in seconds


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
        self.node.get_logger().info("Starting Step Delay Timer...")

    def update(self):
        """
        Create a timer that acts as a delay fro each step,
        Allows steps to not happen too fast so robot can have time to make decisions
        that ahve impact
        
        Returns:
            py_trees.common.Status: SUCCESS if reward calculation is successful, FAILURE otherwise.
        """
        # If reward is not yet available, return RUNNING (waiting for data)
        if self.time_limit_reached == False:
            # self.node.get_logger().info("Waiting to reach time limit...")
            return py_trees.common.Status.RUNNING
        elif self.time_limit_reached == True:
            self.node.get_logger().info(f"Time limit reached.. continue...")
            # Return SUCCESS if reward is available
            return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        """
        This is called when the behaviour switches to a non-running state (SUCCESS, FAILURE, INVALID).
        """
        self.time_limit_reached = False
        # if new_status != py_trees.common.Status.RUNNING:
        #     self.node.get_logger().info(f"Terminating reward calculation with status {new_status}")

    def timer_callback(self):
        """
        Timer Callback
        
        Args:
            msg (StepData): The incoming message containing the reward.
        """
        # Extract reward from the message
        self.time_limit_reached = True
        # self.node.get_logger().info(f"Received step reward: {self.reward}")
