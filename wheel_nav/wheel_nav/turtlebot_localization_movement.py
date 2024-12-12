import rclpy
import py_trees

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.duration import Duration


class MoveAndSpin(py_trees.behaviour.Behaviour):
    def __init__(self, node, duration=10.0, linear_speed=0.2, angular_speed=0.5, safe_distance=0.1):
        super().__init__(name="MoveAndSpinWithObstacleAvoidance")
        self.node = node
        self.duration = Duration(seconds=duration)
        self.linear_speed = linear_speed
        self.angular_speed = angular_speed
        self.safe_distance = safe_distance
        self.publisher = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.start_time = None
        self.phase = "FORWARD"
        self.phase_start_time = None
        self.phase_duration = Duration(seconds=2.0)

        # Obstacle detection
        self.closest_distance = float('inf')
        self.scan_subscriber = self.node.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )

    def scan_callback(self, msg):
        self.closest_distance = min(msg.ranges)

    def initialise(self):
        self.start_time = self.node.get_clock().now()
        self.phase = "FORWARD"
        self.phase_start_time = self.start_time
        self.node.get_logger().info(f"Starting move and spin for {self.duration.nanoseconds / 1e9} seconds")

    def update(self):
        if self.start_time is None:
            return py_trees.common.Status.FAILURE
        
        current_time = self.node.get_clock().now()
        elapsed_time = current_time - self.start_time

        # Check if total duration has elapsed
        if elapsed_time >= self.duration:
            twist = Twist()  # Stop the robot
            self.publisher.publish(twist)
            self.node.get_logger().info("Move and spin sequence complete!")
            return py_trees.common.Status.SUCCESS

        # Check obstacle distance
        if self.closest_distance < self.safe_distance:
            twist = Twist()  # Stop the robot
            self.publisher.publish(twist)
            self.node.get_logger().warn(f"Obstacle detected! Closest distance: {self.closest_distance:.2f} meters")
            return py_trees.common.Status.RUNNING

        # Check if the current phase duration has elapsed
        phase_elapsed_time = current_time - self.phase_start_time
        if phase_elapsed_time >= self.phase_duration:
            # Switch to the next phase
            self.phase_start_time = current_time
            if self.phase == "FORWARD":
                self.phase = "BACKWARD"
            elif self.phase == "BACKWARD":
                self.phase = "SPIN"
            elif self.phase == "SPIN":
                self.phase = "FORWARD"

        # Execute the current phase
        twist = Twist()
        if self.phase == "FORWARD":
            twist.linear.x = self.linear_speed
            self.node.get_logger().info("Moving forward")
        elif self.phase == "BACKWARD":
            twist.linear.x = -self.linear_speed
            self.node.get_logger().info("Moving backward")
        elif self.phase == "SPIN":
            twist.angular.z = self.angular_speed
            self.node.get_logger().info("Spinning in place")

        # Publish the twist message
        self.publisher.publish(twist)
        return py_trees.common.Status.RUNNING
