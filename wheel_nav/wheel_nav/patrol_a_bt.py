import rclpy
from rclpy.node import Node
import py_trees
import py_trees_ros
import py_trees.display

from wheel_nav.navigate_waypoint_state import NavigateWaypoint
from wheel_nav.set_init_pose import SetInitPose
from wheel_nav.turtlebot_localization_spin import SpinInCircles
from wheel_nav.clear_local_costmap_state import ClearLocalCostmap

class BasicBehaviorTree(Node):
    def __init__(self):
        super().__init__('basic_behavior_tree')

        # Localization Behaviors
        set_initial_pose = SetInitPose(self, x=0.2, y=0.2, theta=0.0, cov=0.5**2)
        localization_spin  = SpinInCircles(self, duration=15.0, speed=0.5)
        # clear_local_costmap = ClearLocalCostmap(self, distance=1.0)

        init_localization = py_trees.composites.Sequence("Sequence", memory=True)
        init_localization.add_children([set_initial_pose, localization_spin]) #, clear_local_costmap])


        # Patrol Route 
        waypoints = [
            (0.10, 1.00),  
            (2.50, 3.25),  
            (1.00, 0.30),
            (3.25, 0.25),
            (3.30, 3.25),
        ]
        reference_frame = "map"

        # Create a list of NavigateWaypoint nodes dynamically
        patrol_route_a_waypoints = [NavigateWaypoint(self, reference_frame, wp) for wp in waypoints]

        patrol_route_a = py_trees.composites.Sequence("Patrol Route A", memory=True)
        patrol_route_a.add_children(patrol_route_a_waypoints)
        
        # Root
        self.root = py_trees.composites.Sequence(name="Root", memory=True)
        self.root.add_children([init_localization, patrol_route_a])

        # BT Creation
        self.tree = py_trees_ros.trees.BehaviourTree(self.root)
        self.get_logger().info("Behavior Tree Initialized")

        # Visualize Tree
        py_trees.display.render_dot_tree(self.tree.root)
        # py_trees.display.render_dot_tree(
        #     self.tree.root, 
        #     filename="simple_nav", 
        #     target_directory="/path/to/your/desired/directory",
        #     formats=['pdf', 'png', 'svg']
        # )


    def run_tree(self):
        tree_completed = False  

        while rclpy.ok() and not tree_completed:
            try:
                # Tick the behavior tree
                self.tree.tick()
                root_status = self.root.status
                if root_status == py_trees.common.Status.SUCCESS:
                    self.get_logger().info("Behavior tree completed successfully. Stopping execution.")
                    tree_completed = True 
                rclpy.spin_once(self, timeout_sec=0.1)
            except KeyboardInterrupt:
                break

def main(args=None):
    rclpy.init(args=args)
    behavior_tree_node = BasicBehaviorTree()

    try:
        behavior_tree_node.run_tree()
    except KeyboardInterrupt:
        pass
    finally:
        behavior_tree_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
