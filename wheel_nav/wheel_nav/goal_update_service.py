from wheel_nav_msgs.msg import GoalPosition  
from wheel_nav_msgs.srv import GoalUpdate

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

import random

"""
This is a service that when called will create a publisher that publishes a new waypoint (x,y) to the GoalPosition topic.
Currently, the waypoint is randomly generated from a range of coordinates. In an open space this is fine, however, 
    in an environment with many obstacles, it is not that simple.
Consider generating semi-random waypoints (from a list of valid waypoints) or other methods.
"""

class GoalService(Node):
    def __init__(self):
        super().__init__('goal_service_node')

        # Create a publisher to the GoalPosition topic
        self.goal_pub = self.create_publisher(GoalPosition, 'goal_position', 10)
        
        # Create the service (assuming you have a GoalUpdate.srv)
        self.srv = self.create_service(GoalUpdate, 'goal_update', self.goal_update_callback)

        # List of valid waypoints
        self.goal_range = (-1.8, 1.8)
        self.possible_waypoints = [
            (1.0, 0.0), (-1.5, 1.2), (1.3, -1.1), (0.0, 1.5),
            (-1.0, -1.0), (1.8, -0.5), (-0.2, 1.0), (0.5, 0.5),
            (0.8, -1.5), (-1.2, 0.9), (1.6, 1.4), (-0.8, -1.3)
        ]
        
    def goal_update_callback(self, request, response):
        # Create a GoalPosition message with the new goal coordinates
        goal_msg = GoalPosition()

        # Randomly generate a Goal
        # goal_msg.x = round(random.uniform(self.goal_range[0], self.goal_range[1]), 2)
        # goal_msg.y = round(random.uniform(self.goal_range[0], self.goal_range[1]), 2)

        # Select a waypoint from semi-random list
        selected_waypoint = random.choice(self.possible_waypoints)

        goal_msg.x, goal_msg.y = selected_waypoint
        

        # Publish the new goal position
        self.goal_pub.publish(goal_msg)
        
        # Respond to the service call
        response.success = True
        response.x = goal_msg.x
        response.y = goal_msg.y
        # self.get_logger().info(f"Goal position updated successfully.[{goal_msg.x}, {goal_msg.y}]")
        return response

def main(args=None):
    rclpy.init(args=args)
    goal_service_node = GoalService()
    
    rclpy.spin(goal_service_node)
    
    goal_service_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
