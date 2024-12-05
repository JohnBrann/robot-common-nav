import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
import time 
import transforms3d
import py_trees


class SetInitPose(py_trees.behaviour.Behaviour):
    def __init__(self, node, x=0.1, y=0.1, theta=0.0, cov=0.5**2):
        super().__init__(name="SetInitPose")
        self.node = node  # Pass in the ROS 2 node
        self.x = x
        self.y = y
        self.theta = theta
        self.cov = cov
        self.publisher = None
        self.pose_published = False

        self.publisher = self.node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.node.get_logger().info("SetInitPose behavior setup completed")

    def initialise(self):
        self.pose_published = False  # Reset flag for each tick
        self.node.get_logger().info(f"Initializing publish initial pose: x={self.x}, y={self.y}, theta={self.theta}")

    def update(self):
        if self.publisher is None:
            self.node.get_logger().error("Publisher is not initialized. Ensure `setup()` is called before ticking.")
            return py_trees.common.Status.FAILURE

        if self.pose_published:
            return py_trees.common.Status.SUCCESS

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y

        quat = transforms3d.euler.euler2quat(0, 0, self.theta)
        msg.pose.pose.orientation.w = quat[0]
        msg.pose.pose.orientation.x = quat[1]
        msg.pose.pose.orientation.y = quat[2]
        msg.pose.pose.orientation.z = quat[3]

        msg.pose.covariance = [
            self.cov, 0.0, 0.0, 0.0, 0.0, 0.0,  # Pos X
            0.0, self.cov, 0.0, 0.0, 0.0, 0.0,  # Pos Y
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  # Pos Z
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  # Rot X
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  # Rot Y
            0.0, 0.0, 0.0, 0.0, 0.0, self.cov   # Rot Z
        ]

        # Publish the message
        self.publisher.publish(msg)

        # Status is getting sent too quickly, wait until init pose is fully set
        time.sleep(3)
        self.node.get_logger().info(f"Published initial pose: x={self.x}, y={self.y}, theta={self.theta}")
        self.pose_published = True

        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        if new_status == py_trees.common.Status.SUCCESS:
            self.node.get_logger().info("Successfully published initial pose.")
        elif new_status == py_trees.common.Status.FAILURE:
            self.node.get_logger().error("Failed to publish initial pose.")

