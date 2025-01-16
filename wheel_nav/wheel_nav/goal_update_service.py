from wheel_nav_msgs.msg import GoalPosition  
from wheel_nav_msgs.srv import GoalUpdate

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

import random

class GoalService(Node):
    def __init__(self):
        super().__init__('goal_service_node')
        self.goal_range = (-1.8, 1.8)

        # Create a publisher to the GoalPosition topic
        self.goal_pub = self.create_publisher(GoalPosition, 'goal_position', 10)
        
        # Create the service (assuming you have a GoalUpdate.srv)
        self.srv = self.create_service(GoalUpdate, 'goal_update', self.goal_update_callback)
        
    def goal_update_callback(self, request, response):
        # Logic to handle the goal update request
        # self.get_logger().info(f"Received new goal: ({request.x}, {request.y})")
        
        # Create a GoalPosition message with the new goal coordinates
        goal_msg = GoalPosition()
        # goal_msg.x = request.x
        # goal_msg.y = request.y
        # Randomly generate a Goal, in future we may want to make this semi-randomly generated (random from a list of valid goals)
        # For an open world this is fine
        goal_msg.x = random.uniform(self.goal_range[0], self.goal_range[1])
        goal_msg.y = random.uniform(self.goal_range[0], self.goal_range[1])
        
        # Publish the new goal position
        self.goal_pub.publish(goal_msg)
        
        # Respond to the service call
        response.success = True
        self.get_logger().info(f"Goal position updated successfully.  \n\n                                       [{goal_msg.x}, {goal_msg.y}]\n\n")
        return response

def main(args=None):
    rclpy.init(args=args)
    goal_service_node = GoalService()
    
    rclpy.spin(goal_service_node)
    
    goal_service_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
