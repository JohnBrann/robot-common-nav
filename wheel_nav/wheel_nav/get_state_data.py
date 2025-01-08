import rclpy
import py_trees
import py_trees_ros

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

        # Subscribe to the StepData topic
        self.subscription = self.node.create_subscription(
            StepData,
            'step_data',
            self.listener_callback,
            10
        )
        # self.subscription  # prevent unused variable warning # do we need? 

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
        # self.node.get_logger().info("Calculating reward from current state...")

    def update(self):
        """
        Set node specific variable to state data 
        
        Returns:
            py_trees.common.Status: SUCCESS if reward calculation is successful, FAILURE otherwise.
        """

        if self.callback_called == False:
            self.node.get_logger().info("Waiting for state data...")
            return py_trees.common.Status.RUNNING
        


        self.node.distance_to_goal = self.distance_to_goal
        self.node.angle_to_goal = self.angle_to_goal
        self.node.scan_data = self.scan_data
        self.node.min_obstacle_distance = self.min_obstacle_distance
        self.node.angular_velocity = self.angular_velocity
        self.node.linear_velocity = self.linear_velocity
        self.node.terminated = self.terminated
        self.node.success = self.success
        self.node.reward = self.reward

        # self.node.get_logger().info(f"Distance to goal: {self.node.distance_to_goal}")
        # self.node.get_logger().info(f"Angle to goal: {self.node.angle_to_goal}")
        # self.node.get_logger().info(f"Scan Data: {self.node.scan_data}")
        # self.node.get_logger().info(f"min_obstacle_distance: {self.node.min_obstacle_distance}")
        # self.node.get_logger().info(f"angular_velocity: {self.node.angular_velocity}")
        # self.node.get_logger().info(f"linear_velocity: {self.node.linear_velocity}")
        # self.node.get_logger().info(f"terminated: {self.node.success}")
        # self.node.get_logger().info(f"Step Reward: {self.node.reward}")

        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        """
        This is called when the behaviour switches to a non-running state (SUCCESS, FAILURE, INVALID).
        """

        self.callback_called = False
        # self.reward = None
        # if new_status != py_trees.common.Status.RUNNING:
        #     self.node.get_logger().info(f"Terminating reward calculation with status {new_status}")

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

        self.node.get_logger().info("callback called...")

        # self.node.get_logger().info(f"Distance to goalllllllllll: {self.distance_to_goal}")
        # self.node.get_logger().info(f"Step Reward: {self.reward}")

