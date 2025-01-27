import rclpy 
import py_trees
import random
import numpy as np

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

    def initialise(self):
        """
        This is called the first time the behaviour is ticked and anytime the status is not RUNNING thereafter.
        """
        
    def update(self):
        """
        In the future, this will check the discrete or continuous parameter, or there will be another ndoe just for the discrete parameter
        
        Returns:
            py_trees.common.Status: SUCCESS if training is enabled, FAILURE otherwise
        """

        # Define action bounds
        linear_velocity_low = 0.0
        linear_velocity_high = 0.2
        angular_velocity_low = -1.0
        angular_velocity_high = 1.0

        # Get the action from the actor network (ensure it's compatible with numpy)
        state = self.node.state  # The current state must be provided
        action = self.node.actor(state).cpu().data.numpy().flatten()

        # Scale and clip the actions to their valid ranges
        linear_velocity = np.clip(action[0], linear_velocity_low, linear_velocity_high)
        angular_velocity = np.clip(action[1], angular_velocity_low, angular_velocity_high)

        # Add noise for exploration (during training)
        if self.epsilon > 0.0:
            linear_velocity += np.random.uniform(-self.epsilon, self.epsilon)
            angular_velocity += np.random.uniform(-self.epsilon, self.epsilon)

            # Clip again after adding noise
            linear_velocity = np.clip(linear_velocity, linear_velocity_low, linear_velocity_high)
            angular_velocity = np.clip(angular_velocity, angular_velocity_low, angular_velocity_high)

        # Publish the action
        self.publish_twist(linear_velocity, angular_velocity)

        return py_trees.common.Status.SUCCESS
        
    def terminate(self, new_status):
        """
        This is called when the behaviour switches to a non-running state.
            SUCCESS || FAILURE || INVALID
        """

    def publish_twist(self, linear_velocity, angular_velocity):
        """
        Publish the selected action as a Twist message.
        
        Args:
            linear_velocity (float): Linear velocity for the TurtleBot
            angular_velocity (float): Angular velocity for the TurtleBot
        """
        twist = Twist()
        twist.linear.x = linear_velocity
        twist.angular.z = angular_velocity
        self.publisher.publish(twist)
        self.node.get_logger().info(
            f"Published action: linear_velocity = {linear_velocity:.2f}, angular_velocity = {angular_velocity:.2f}"
        )