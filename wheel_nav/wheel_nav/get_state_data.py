import rclpy
import py_trees
import py_trees_ros
import numpy as np
import torch
import math

from wheel_nav_msgs.msg import StepData

class GetStateData(py_trees.behaviour.Behaviour):
    def __init__(self, node, name):
        """
        Initialize the condition node with a reference to the ROS 2 node
        
        Args:
            name (str): Name of the behavior
            node (rclpy.node.Node): ROS 2 node to access parameters
        """
        super().__init__(name)
        self.node = node

        self.callback_called = False

        # initialize state data variables
        self.distance_to_goal = None
        self.angle_to_goal = None
        self.scan_data = []
        self.min_obstacle_distance = None 
        self.angular_velocity = None
        self.linear_velocity = None
        self.terminated = None
        self.success = None
        self.reward = None

        # Normalization constants
        # These values should become dynamic to the environment in the future
        self.MAX_GOAL_DISTANCE = 5.0  # Example max distance to goal in meters
        self.MAX_SCAN_DISTANCE = 3.5  # Example max range of LiDAR in meters
        self.MAX_VELOCITY = 1.0       # Example max velocity in m/s

        # Subscribe to the StepData topic
        self.subscription = self.node.create_subscription(
            StepData,
            'step_data',
            self.listener_callback,
            10
        )
        # self.subscription  # prevent unused variable warning

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
        Set node specific variable to state data

        Data is normalized from [0,1]. This is common practice in RL
        
        Returns:
            py_trees.common.Status: SUCCESS if reward calculation is successful, FAILURE otherwise.
        """

        if self.callback_called == False:
            # self.node.get_logger().info("Waiting for state data...")
            return py_trees.common.Status.RUNNING
        
        self.node.distance_to_goal = self.normalize_distance(self.distance_to_goal)
        self.node.angle_to_goal = self.normalize_angle(self.angle_to_goal)
        self.node.scan_data = self.normalize_scan_data(self.scan_data)
        self.node.min_obstacle_distance = self.normalize_distance(self.min_obstacle_distance)
        self.node.angular_velocity = self.normalize_velocity(self.angular_velocity)
        self.node.linear_velocity = self.normalize_velocity(self.linear_velocity)
        self.node.terminated = self.terminated
        self.node.success = self.success
        self.node.reward = self.reward

        # Log normalized data
        # self.node.get_logger().info(f"Normalized distance to goal: {self.node.distance_to_goal}")
        # self.node.get_logger().info(f"Normalized angle to goal: {self.node.angle_to_goal}")
        # self.node.get_logger().info(f"Normalized scan data: {self.node.scan_data}")
        # self.node.get_logger().info(f"Normalized min obstacle distance: {self.node.min_obstacle_distance}")
        # self.node.get_logger().info(f"Normalized angular velocity: {self.node.angular_velocity}")
        # self.node.get_logger().info(f"Normalized linear velocity: {self.node.linear_velocity}")

        # Ensure scan_data is a NumPy array before concatenation
        scan_data_array = np.array(self.node.scan_data)

        # Consolidate all data into a single state
        np_state = np.concatenate((
            scan_data_array,  # LiDAR scan data as a NumPy array
            np.array([
                self.node.distance_to_goal,         # Normalized distance to goal [0, 1]
                self.node.angle_to_goal,            # Normalized angle to goal [-1, 1]
                self.node.min_obstacle_distance,    # Normalized min obstacle distance [0, 1]
                self.node.angular_velocity,         # Normalized angular velocity [-1, 1]
                self.node.linear_velocity,          # Normalized linear velocity [-1, 1]
            ])
        ))

        # Convert state to a PyTorch tensor
        state_tensor = torch.tensor(np_state, dtype=torch.float)

        # Add a batch dimension for PyTorch (if needed for the NN)
        state_tensor = state_tensor.unsqueeze(dim=0)

        # Move tensor to the appropriate device (e.g., GPU if available)
        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        state_tensor = state_tensor.to(device)

        # Store tensor for later use
        self.node.new_state = state_tensor

        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        """
        This is called when the behaviour switches to a non-running state (SUCCESS, FAILURE, INVALID).
        """
        self.callback_called = False

    def listener_callback(self, msg):
        """
        Callback to get state data to append to be used at the start of the episode
        
        Args:
            msg (StepData): The incoming message containing the reward.
        """

        self.distance_to_goal = msg.distance_to_goal
        self.angle_to_goal = msg.angle_to_goal
        self.scan_data = msg.scan_data
        self.min_obstacle_distance = msg.min_obstacle_distance
        self.angular_velocity = msg.angular_velocity
        self.linear_velocity = msg.linear_velocity
        self.terminated = msg.terminated
        self.success = msg.success
        self.reward = msg.reward

        self.callback_called = True

    def normalize_distance(self, distance):
        """Normalize distance to the range [0, 1]."""
        return min(max(distance / self.MAX_GOAL_DISTANCE, 0.0), 1.0)

    def normalize_angle(self, angle):
        """Normalize angle to the range [-1, 1]."""
        return min(max(angle / math.pi, -1.0), 1.0)

    def normalize_scan_data(self, scan_data):
        """Normalize LiDAR scan data to the range [0, 1]."""
        return [min(max(d / self.MAX_SCAN_DISTANCE, 0.0), 1.0) for d in scan_data]

    def normalize_velocity(self, velocity):
        """Normalize velocity to the range [-1, 1]."""
        return min(max(velocity / self.MAX_VELOCITY, -1.0), 1.0)