import rclpy
import py_trees
import py_trees_ros

from wheel_nav_msgs.msg import StepData

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

        self.distance_to_goal = None
        self.angle_to_goal = None
        self.scan_data = None
        self.min_obstacle_distance = None 
        self.angular_velocity = None
        self.linear_velocity = None
        self.terminated = None
        self.reward = None

        # Subscribe to the StepData topic
        # self.subscription = self.node.create_subscription(
        #     StepData,
        #     'step_data',
        #     self.listener_callback,
        #     10
        # )
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
        Calculate the current reward given the current state of the robot.
        
        Returns:
            py_trees.common.Status: SUCCESS if reward calculation is successful, FAILURE otherwise.
        """

        self.node.memory.append((
            self.node.distance_to_goal,
            self.node.angle_to_goal,
            self.node.scan_data,
            self.node.min_obstacle_distance,
            self.node.angular_velocity,
            self.node.linear_velocity,
            self.node.terminated,
            self.node.reward
        ))

        # sample = self.node.memory.sample(1) 
        # self.node.get_logger().info(f"Sampled Transitions: {sample}")


        # Return SUCCESS if reward is available
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        """
        This is called when the behaviour switches to a non-running state (SUCCESS, FAILURE, INVALID).
        """
        # if new_status != py_trees.common.Status.RUNNING:
        #     self.node.get_logger().info(f"Terminating reward calculation with status {new_status}")

    # def listener_callback(self, msg):
    #     """
    #     Callback to get data to append to memory
        
    #     Args:
    #         msg (StepData): The incoming message containing the reward.
    #     """

    #     self.distance_to_goal = msg.distance_to_goal
    #     self.angle_to_goal = msg.angle_to_goal
    #     self.scan_data = msg.scan_data
    #     self.min_obstacle_distance = msg.min_obstacle_distance
    #     self.angular_velocity = msg.angular_velocity
    #     self.linear_velocity = msg.linear_velocity
    #     self.terminated = msg.terminated
    #     self.reward = msg.reward
