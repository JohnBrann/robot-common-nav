import rclpy 
import py_trees
import random

from geometry_msgs.msg import Twist


class SelectContinuousAction(py_trees.behaviour.Behaviour):
    def __init__(self, node, name):
        """
        Initialize the condition node with a reference to the ROS 2 node
        
        Args:
            name (str): Name of the behavior
            node (rclpy.node.Node): ROS 2 node to access parameters
        """
        super().__init__(name)
        self.node = node
        self.epsilon = 0.0
        self.publisher = self.node.create_publisher(Twist, '/cmd_vel', 10)
        

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
        self.node.get_logger().info(f"Selecting action...")
        

    def update(self):
        """
        In the future, this will check the discrete or continuous parameter, or there will be another ndoe just for the discrete parameter
        
        Returns:
            py_trees.common.Status: SUCCESS if training is enabled, FAILURE otherwise
        """

        # discrete_actions = [0, 1, 2, 3, 4]

        # if random.random.sample() < self.epsilon:
        #     selected_action = random.choice(discrete_actions)
        # else:
        #     # select action from the NN
        #     selected_action = 2


        # selected_action = 2

        # self.publish_action(1)

        return py_trees.common.Status.SUCCESS
        
    def terminate(self, new_status):
        """
        This is called when the behaviour switches to a non-running state.
            SUCCESS || FAILURE || INVALID
        """
        # self.node.get_logger().info(f"Terminating TrainingModeState with status {new_status}")

    def publish_twist(self, selected_action):

        twist = Twist()


        twist.linear.x = 0.1
        twist.angular.z = -1.0

        self.publisher.publish(twist)