import rclpy
import py_trees
from py_trees.behaviour import Behaviour
from wheel_nav_msgs.srv import Waypoint

class NavigateWaypoint(Behaviour):
    def __init__(self, node, reference_frame, waypoint):
        super().__init__(name=f"NavigateWaypoint {waypoint}")
        self.node = node
        self.reference_frame = reference_frame
        self.waypoint = waypoint
        self.client = self.node.create_client(Waypoint, "waypoint")
        self.future = None

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info("Waiting for the waypoint service to become available...")

    def initialise(self):
        self.node.get_logger().info(f"Initializing navigation to waypoint {self.waypoint}")
        req = Waypoint.Request()
        req.pose.position.x, req.pose.position.y, req.pose.position.z = self.waypoint[0], self.waypoint[1], 0.0
        req.pose.orientation.w = 1.0
        req.reference_frame_id = self.reference_frame
        self.future = self.client.call_async(req)
        self.node.get_logger().info(f"Navigation to {self.waypoint} is in progress...")
         
    def update(self):
        if self.future is None:
            self.node.get_logger().error("Future is None; service was not initialized properly.")
            return py_trees.common.Status.FAILURE

        if not self.future.done():
            return py_trees.common.Status.RUNNING

        try:
            response = self.future.result()
            if response.result == "SUCCESS":
                self.node.get_logger().info(f"Navigation to {self.waypoint} succeeded")
                return py_trees.common.Status.SUCCESS
            else:
                self.node.get_logger().error(f"Navigation to {self.waypoint} failed with result: {response.result}")
                return py_trees.common.Status.FAILURE
        except Exception as e:
            self.node.get_logger().error(f"Service call failed: {e}")
            return py_trees.common.Status.FAILURE


    # def terminate(self, new_status):
    #     self.node.get_logger().info(f"Terminating navigation to {self.waypoint} with status: {new_status}")
    #     if new_status in {Status.SUCCESS, Status.FAILURE}:
    #         self.future = None