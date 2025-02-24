import rclpy 
import py_trees
import random
import torch
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
        linear_velocity_high = 0.22
        angular_velocity_low = -1.0
        angular_velocity_high = 1.0

        # Convert state to PyTorch tensor & ensure correct shape
        state = torch.tensor(self.node.state, dtype=torch.float32, device=self.node.device).unsqueeze(0)  # Shape: [1, state_dim]

        # print(f'state shape before: {state.shape}')

        # Ensure state is 2D (batch_size=1, state_dim)
        if state.dim() == 1:
            state = state.unsqueeze(0)  # Shape: [1, state_dim]
        elif state.dim() == 3:  # If extra dimension exists, squeeze it
            state = state.squeeze(0)  # Ensure shape is [1, state_dim]

        # print(f'state shape after: {state.shape}')


        # Get action from the actor network
        # Get action from the actor network

        # print(f'STATTTTTTTTTTEEEEEEEEEEEEEEEEE PASSSSSSSSSSSEDDDDDDDDDDDDDDDDDDD: {state}')
        action = self.node.actor(state)  # Output shape: [1, 2] (linear_velocity, angular_velocity)

        # Ensure action shape is correct
        if action.shape[1] != 2:
            raise ValueError(f"Expected action shape [1, 2], but got {action.shape}")

        # Define separate noise levels for linear and angular velocity
        linear_noise_std = 0.1  # Small noise for linear velocity (10% of max range)
        angular_noise_std = 0.1  # Larger noise for angular velocity (10% of max range)

        # Create noise tensor
        noise = torch.tensor(np.random.normal(0, [linear_noise_std, angular_noise_std], size=(1, 2)), 
                            dtype=torch.float32, device=self.node.device)
        
        # self.node.get_logger().info(f"Before Noise Action: [{action}]")
        self.node.get_logger().info(f"Noise: [{noise}]")

        # Add noise before applying bounds
        action = action + noise
        # self.node.get_logger().info(f"After Noise Action: [{action}]")

        # Clamp actions within valid bounds
        action[:, 0] = torch.clamp(action[:, 0], linear_velocity_low, linear_velocity_high)  # Linear velocity [0, 0.22]
        action[:, 1] = torch.clamp(action[:, 1], angular_velocity_low, angular_velocity_high)  # Angular velocity [-1, 1]

        # Convert to NumPy and extract values
        action = action.squeeze(0).cpu().data.numpy()  # Shape: (2,)
        linear_velocity, angular_velocity = action[0], action[1]

        # Add additional exploration noise (during training)
        # if self.node.epsilon > 0.45:
        #     # linear_velocity += np.random.uniform(-self.epsilon, self.epsilon)
        #     angular_velocity += np.random.uniform(-self.node.epsilon, self.node.epsilon)

        #     # self.node.get_logger().info(f'EPSILONNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNN')

        #     # Clip again after noise
        #     # linear_velocity = np.clip(linear_velocity, linear_velocity_low, linear_velocity_high)
        #     angular_velocity = np.clip(angular_velocity, angular_velocity_low, angular_velocity_high)

        self.node.get_logger().info(f"Action Selected by NN:  [{linear_velocity}][{angular_velocity}]")

        selected_actions = np.array([linear_velocity, angular_velocity])
        self.node.action = torch.tensor(selected_actions, dtype=torch.float32)


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
        twist.linear.x = float(linear_velocity)
        twist.angular.z = float(angular_velocity)
        self.publisher.publish(twist)
        # self.node.get_logger().info(
        #     f"Published action: linear_velocity = {linear_velocity:.2f}, angular_velocity = {angular_velocity:.2f}"
        # )