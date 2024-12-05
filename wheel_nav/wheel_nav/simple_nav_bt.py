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


        # Navigation Route A
        a1 = (0.1, 1.0)
        a2 = (0.1, 2.0)
        a3 = (1.0, 2.0)
        reference_frame = "map"

        waypoint_a1 = NavigateWaypoint(self, reference_frame, a1)
        waypoint_a2 = NavigateWaypoint(self, reference_frame, a2)
        waypoint_a3 = NavigateWaypoint(self, reference_frame, a3)

        route_a = py_trees.composites.Sequence("Sequence", memory=True)
        route_a.add_children([waypoint_a1, waypoint_a2, waypoint_a3])
        
        # Root
        self.root = py_trees.composites.Sequence(name="Root", memory=True)
        self.root.add_children([init_localization, route_a])

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
