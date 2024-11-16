from navigation_msgs.srv import Waypoint
from geometry_msgs.msg import Pose
import rclpy
from rclpy.node import Node


class WaypointClient(Node):

    def __init__(self):
        super().__init__('waypoint_service')
        self.waypoint_client = self.create_client(Waypoint, "waypoint")

        # Wait for the service to become available
        while not self.waypoint_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for the waypoint service to become available...")

    def send_waypoint_request(self, x, y, reference_frame):
        """
        Send a waypoint request to the service with the provided coordinates and reference frame.
        """
        req = Waypoint.Request()

        # Fill the request with the given waypoint
        req.pose.position.x = x
        req.pose.position.y = y
        req.pose.position.z = 0.0
        req.pose.orientation.x = 0.0
        req.pose.orientation.y = 0.0
        req.pose.orientation.z = 0.0
        req.pose.orientation.w = 1.0
        req.reference_frame_id = reference_frame

        # Send the request asynchronously
        self.get_logger().info(f"Sending waypoint: Pose=({x}, {y}, 0.0), Frame={reference_frame}")
        future = self.waypoint_client.call_async(req)

        # Wait for the response
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f"Received response: Result=({future.result().result})")
        else:
            self.get_logger().error("Failed to receive response from the server.")


def main():
    rclpy.init()

    # Create the client node
    waypoint_client = WaypointClient()

    try:
        # Run a loop to continuously accept waypoints
        while rclpy.ok():
            print("\nEnter waypoint details (or type 'exit' to quit):")

            # Get user input
            x_input = input("Enter X coordinate: ")
            if x_input.lower() == 'exit':
                break

            y_input = input("Enter Y coordinate: ")
            if y_input.lower() == 'exit':
                break

            reference_frame = input("Enter reference frame (e.g., 'map'): ")
            if reference_frame.lower() == 'exit':
                break

            # Validate inputs
            try:
                x = float(x_input)
                y = float(y_input)
            except ValueError:
                print("Invalid coordinates. Please enter numeric values for X and Y.")
                continue

            # Send the waypoint request
            waypoint_client.send_waypoint_request(x, y, reference_frame)

    except KeyboardInterrupt:
        print("\nShutting down client.")

    # Cleanup
    waypoint_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
