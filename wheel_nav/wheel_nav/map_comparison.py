import rclpy
from rclpy.node import Node
from wheel_nav_msgs.srv import CompareMaps
from nav_msgs.msg import OccupancyGrid
import numpy as np

class MapComparisonNode(Node):
    def __init__(self):
        super().__init__('map_comparison_service')
        self.global_map = None
        self.local_costmap = None

        # Subscribers
        self.create_subscription(OccupancyGrid, '/map', self.global_map_callback, 10)
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap', self.local_costmap_callback, 10)
        
        # Service
        self.srv = self.create_service(CompareMaps, '/compare_maps', self.compare_maps_callback)

    def global_map_callback(self, msg):
        self.global_map = msg
        self.get_logger().info(f"Global Map - Width: {msg.info.width}, Height: {msg.info.height}")

    def local_costmap_callback(self, msg):
        self.local_costmap = msg
        self.get_logger().info(f"Local Costmap - Width: {msg.info.width}, Height: {msg.info.height}")

    def compare_maps_callback(self, request, response):
        # Check if both maps are available
        if not self.global_map or not self.local_costmap:
            self.get_logger().warn("Maps not available")
            response.is_aligned = False
            response.alignment_score = 0.0
            return response

        # Convert to numpy arrays
        global_map_array = np.array(self.global_map.data).reshape(
            (self.global_map.info.height, self.global_map.info.width)
        )
        local_costmap_array = np.array(self.local_costmap.data).reshape(
            (self.local_costmap.info.height, self.local_costmap.info.width)
        )

        # Resizing/Alignment Check
        if global_map_array.shape != local_costmap_array.shape:
            self.get_logger().warn("Map sizes do not match. Attempting to align...")
            # Add resizing logic if needed
            response.is_aligned = False
            response.alignment_score = 0.0
            return response

        # Comparison Logic
        # Compare occupied spaces
        global_occupied = global_map_array > 50  # Threshold for occupied space
        local_occupied = local_costmap_array > 50

        # Calculate overlap
        total_cells = global_map_array.size
        matching_occupied_cells = np.sum(global_occupied == local_occupied)
        
        alignment_score = (matching_occupied_cells / total_cells) * 100

        # Set response
        response.is_aligned = alignment_score > 70.0  # Configurable threshold
        response.alignment_score = alignment_score

        self.get_logger().info(f"Alignment Score: {alignment_score}%")
        return response

def main(args=None):
    rclpy.init(args=args)
    node = MapComparisonNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


# import rclpy
# from rclpy.node import Node
# from wheel_nav_msgs.srv import CompareMaps
# from nav_msgs.msg import OccupancyGrid
# from std_srvs.srv import Trigger
# import numpy as np

# class MapComparisonNode(Node):
#     def __init__(self):
#         super().__init__('map_comparison_service')
#         self.srv = self.create_service(CompareMaps, '/compare_maps', self.compare_maps_callback)
#         self.get_logger().info("Map Comparison Service Initialized")
        
#         # Subscribers to the costmap and global map topics
#         self.global_map = None
#         self.local_costmap = None
        
#         self.create_subscription(OccupancyGrid, '/map', self.global_map_callback, 10)
#         self.create_subscription(OccupancyGrid, '/local_costmap/costmap', self.local_costmap_callback, 10)
    
#     def global_map_callback(self, msg):
#         self.global_map = msg
    
#     def local_costmap_callback(self, msg):
#         self.local_costmap = msg
    
#     def compare_costmap_with_map(self, global_map, local_costmap):
#         # assume both maps are of the same size and resolution
#         # Compare the occupancy values cell by cell (could improve by accounting for tolerance)
#         matching_cells = np.sum(global_map == local_costmap)
#         total_cells = global_map.size
#         alignment_score = (matching_cells / total_cells) * 100 # percentage of matching cells
#         return alignment_score
    
#     def compare_maps_callback(self, request, response):
#         # Check if maps are available
#         if self.global_map is None or self.local_costmap is None:
#             self.get_logger().warn("Maps not available yet")
#             response.is_aligned = False
#             response.alignment_score = 0.0
#             return response
        
#         # Convert map data to numpy arrays for easier comparison
#         global_map_array = np.array(self.global_map.data).reshape((self.global_map.info.height, self.global_map.info.width))
#         local_costmap_array = np.array(self.local_costmap.data).reshape((self.local_costmap.info.height, self.local_costmap.info.width))
        
#         # Check if maps have same size
#         if global_map_array.shape != local_costmap_array.shape:
#             self.get_logger().warn("Map sizes do not match")
#             response.is_aligned = False
#             response.alignment_score = 0.0
#             return response
        
#         # Compare the maps
#         alignment_score = self.compare_costmap_with_map(global_map_array, local_costmap_array)
#         response.is_aligned = alignment_score > 90.0 # Example threshold (90% match)
#         response.alignment_score = alignment_score
        
#         return response

# def main(args=None):
#     rclpy.init(args=args)
#     map_comparison_node = MapComparisonNode()
#     rclpy.spin(map_comparison_node)
#     map_comparison_node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()