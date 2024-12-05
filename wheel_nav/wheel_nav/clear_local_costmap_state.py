import rclpy
import py_trees

from nav2_msgs.srv import ClearCostmapAroundRobot
from nav2_simple_commander.robot_navigator import BasicNavigator

class ClearLocalCostmap(py_trees.behaviour.Behaviour):
    def __init__(self, node, distance=1):
        super().__init__(name="clear_costmap_client")
        self.node = node
        self.clear_distance = distance
        self.local_costmap_cleared = False
        

        self.client = self.node.create_client(ClearCostmapAroundRobot, '/local_costmap/clear_around_local_costmap')
        self.future = None
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Waiting for ClearCostmapAroundRobot service to become available...')

    def initialise(self):
        self.node.get_logger().info(f"Clearing Costmap around Robot")
        req = ClearCostmapAroundRobot.Request()
        req.reset_distance = self.clear_distance
        self.future = self.client.call_async(req)
        self.node.get_logger().info(f"Clearing Costmap around Robot in progress...")
        # rclpy.spin_until_future_complete(self, self.future)
        self.local_costmap_cleared = self.future.result()


    # def send_request(self, reset_distance):
    #     req = ClearCostmapAroundRobot.Request()
    #     req.reset_distance = reset_distance
    #     self.future = self.cli.call_async(req)
    #     rclpy.spin_until_future_complete(self, self.future)
    #     return self.future.result()
    

    def update(self):
        if self.future is None:
            self.node.get_logger().error("Future is None; service was not initialized properly.")
            return py_trees.common.Status.FAILURE
        
        if self.local_costmap_cleared == False:
            return py_trees.common.Status.RUNNING
        
        try:
            self.local_costmap_cleared = self.future.result()
            if self.local_costmap_cleared == "SUCCESS":
                self.node.get_logger().info(f"Clearing Costmap around Robot succeeded")
                return py_trees.common.Status.SUCCESS
            else:
                self.node.get_logger.info(f"Clearing Costmap around Robot FAILED")
                return py_trees.common.Status.FAILURE
        except Exception as e:
            self.node.get_logger().error(f"Service call failed: {e}")
            return py_trees.common.Status.FAILURE  
        



    # def terminate(self, new_status):
    #     self.node.get_logger().info(f"Terminating Clear Local Costmap")
    #     if new_status in {Status.SUCCESS, Status.FAILURE}:
    #         self.future = None
    

# def main(args=None):
#     rclpy.init(args=args)

#     clear_costmap_client = ClearCostmapClient()
#     response = clear_costmap_client.send_request(1.0) 

#     clear_costmap_client.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()