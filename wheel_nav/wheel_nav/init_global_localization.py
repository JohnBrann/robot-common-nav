import rclpy
from std_srvs.srv import Empty
import py_trees

class GlobalLocalization(py_trees.behaviour.Behaviour):
    def __init__(self, node):
        super().__init__(name="GlobalLocalization")
        self.node = node  
        self.client = None
        self.future = None

        # Create a service client for /reinitialize_global_localization
        self.client = self.node.create_client(Empty, '/reinitialize_global_localization')
        if not self.client.wait_for_service(timeout_sec=10.0):
            self.node.get_logger().error("Service /reinitialize_global_localization not available.")
            raise RuntimeError("Failed to initialize GlobalLocalization behavior")

        self.node.get_logger().info("GlobalLocalization behavior setup completed")

    def initialise(self):
        self.node.get_logger().info("Calling /reinitialize_global_localization service...")
        self.future = self.client.call_async(Empty.Request())

    def update(self):
        if self.future is None:
            self.node.get_logger().error("Service call future is not initialized.")
            return py_trees.common.Status.FAILURE

        # Check if the service call is complete
        if self.future.done():
            try:
                self.future.result()  # Ensure no exceptions were raised
                self.node.get_logger().info("Successfully called /reinitialize_global_localization.")
                return py_trees.common.Status.SUCCESS
            except Exception as e:
                self.node.get_logger().error(f"Service call failed: {e}")
                return py_trees.common.Status.FAILURE

        # Service call is still pending
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        if new_status == py_trees.common.Status.SUCCESS:
            self.node.get_logger().info("Global localization reinitialization completed.")
        elif new_status == py_trees.common.Status.FAILURE:
            self.node.get_logger().error("Global localization reinitialization failed.")
