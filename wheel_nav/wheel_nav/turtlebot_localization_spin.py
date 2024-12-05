import rclpy
import py_trees

from geometry_msgs.msg import Twist
from rclpy.duration import Duration


class SpinInCircles(py_trees.behaviour.Behaviour):
    def __init__(self, node, duration=5.0, speed=0.5):
        super().__init__(name="SpinInCircles")
        self.node = node
        self.duration = Duration(seconds=duration)  # Convert to ROS2 Duration
        self.speed = speed  # Angular speed in radians per second
        self.publisher = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.start_time = None

    def initialise(self):
        self.start_time = self.node.get_clock().now()
        self.node.get_logger().info(f"Starting spin for {self.duration.nanoseconds / 1e9} seconds at speed {self.speed}")

    def update(self):
        if self.start_time is None:
            return py_trees.common.Status.FAILURE
        
        # Calculate the time elapsed
        current_time = self.node.get_clock().now()
        elapsed_time = current_time - self.start_time

        if elapsed_time < self.duration:
            twist = Twist()
            twist.angular.z = self.speed  # Set angular velocity to spin in place
            self.publisher.publish(twist)
            # self.node.get_logger().info(f"Spinning: {elapsed_time.nanoseconds / 1e9:.2f} seconds elapsed")
            return py_trees.common.Status.RUNNING
        else:
            # Stop the robot after the spin is complete
            twist = Twist()
            self.publisher.publish(twist)
            self.node.get_logger().info("Spin complete!")
            return py_trees.common.Status.SUCCESS  # Return success when the spin is finished

    # def terminate(self, new_status):
    #     if new_status == Status.SUCCESS:
    #         self.node.get_logger().info("Spin completed successfully.")
    #     elif new_status == Status.FAILURE:
    #         self.node.get_logger().error("Spin failed.")

