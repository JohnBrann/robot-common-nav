import rclpy 
import py_trees
import random

from geometry_msgs.msg import Twist


class SelectDiscreteAction(py_trees.behaviour.Behaviour):
    def __init__(self, node, name):
        """
        Initialize the condition node with a reference to the ROS 2 node
        
        Args:
            name (str): Name of the behavior
            node (rclpy.node.Node): ROS 2 node to access parameters
        """
        super().__init__(name)
        self.node = node
        # self.epsilon = 0.2 # 0 for testing purposes
        self.publisher = self.node.create_publisher(Twist, '/cmd_vel', 10) # make this into a service call? seperate into another behavior?
        

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
        # self.node.get_logger().info(f"Selecting action...")
        

    def update(self):
        """
        Selects a discrete action, exploration and exploitation
        
        Returns:
            py_trees.common.Status: SUCCESS if action is selected, FAILURE otherwise
        """

        discrete_actions = [0, 1, 2, 3, 4]

        if random.random() < self.node.epsilon:
            selected_action = random.choice(discrete_actions)
            self.node.action = selected_action
            self.node.get_logger().info(f"Random Action Selected: [{selected_action}]") 
        else:
            # select action from the NN
            selected_action = self.node.policy_dqn(self.node.state.unsqueeze(dim=0)).squeeze().argmax()
            # selected_action = 2

            self.node.action = selected_action
            self.node.get_logger().info(f"Action Selected by NN:  [{selected_action}]")

        if self.node.epsilon > 0.1:
            self.node.epsilon = self.node.epsilon * 0.9999

        self.publish_twist(selected_action)

        return py_trees.common.Status.SUCCESS
        
    def terminate(self, new_status):
        """
        This is called when the behaviour switches to a non-running state.
            SUCCESS || FAILURE || INVALID
        """
        # self.node.get_logger().info(f"Terminating TrainingModeState with status {new_status}")

    def publish_twist(self, selected_action):

        twist = Twist()

        if selected_action == 0:
            # self.node.get_logger().info("Selected 0: turning hard left")
            twist.linear.x = 0.2
            twist.angular.z = 1.0
        elif selected_action == 1:
            # self.node.get_logger().info("Selected 1: turning slight left")
            twist.linear.x = 0.2
            twist.angular.z = 0.3
        elif selected_action == 2:
            # self.node.get_logger().info("Selected 2: going straight")
            twist.linear.x = 0.2
            twist.angular.z = 0.0
        elif selected_action == 3:
            # self.node.get_logger().info("Selected 3: turning slight right")
            twist.linear.x = 0.2
            twist.angular.z = -0.3
        elif selected_action == 4:
            # self.node.get_logger().info("Selected 4: turning hard right")
            twist.linear.x = 0.2
            twist.angular.z = -1.0

        self.publisher.publish(twist)