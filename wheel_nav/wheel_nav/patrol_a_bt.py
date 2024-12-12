import rclpy
from rclpy.node import Node
import py_trees
import py_trees_ros
import py_trees.display

from wheel_nav.navigate_waypoint_state import NavigateWaypoint
from wheel_nav.set_init_pose import SetInitPose
from wheel_nav.turtlebot_localization_spin import SpinInCircles
from wheel_nav.clear_local_costmap_state import ClearLocalCostmap
from wheel_nav.init_global_localization import GlobalLocalization
from wheel_nav.turtlebot_localization_movement import MoveAndSpin

class ComplexBehaviorTree(Node):
    def __init__(self):
        super().__init__('complex_behavior_tree')

        # Localization Behaviors (executed once at the beginning)
        set_initial_pose = SetInitPose(self, x=2.5, y=3.0, theta=0.0, cov=0.5**2)
        global_localization = GlobalLocalization(self)
        localization_movement = MoveAndSpin(self, duration=30.0, linear_speed=0.2, angular_speed=0.2, safe_distance=0.1)

        # Local costmap clearing behavior for recovery (to use in case of obstacles or stuck state)
        clear_local_costmap = ClearLocalCostmap(self, distance=1.0)
        spin_in_circles = SpinInCircles(self, duration=10.0, speed=0.2)  # Spin to help recovery

        # Sequence to initialize localization (only runs once at the beginning)
        init_localization = py_trees.composites.Sequence("Initialize Localization", memory=True)
        init_localization.add_children([global_localization, localization_movement])

        # Patrol Route with dynamic waypoints
        waypoints = [
            (0.10, 1.00),  
            (2.50, 3.25),  
            (1.00, 0.30),
            (3.25, 0.25),
            (3.30, 3.25),
        ]
        reference_frame = "map"
        patrol_route_waypoints = [NavigateWaypoint(self, reference_frame, wp) for wp in waypoints]
        patrol_route = py_trees.composites.Sequence("Patrol Route", memory=True)
        patrol_route.add_children(patrol_route_waypoints)

        # **Recovery Sub-tree** (Handling stuck state and retrying navigation)
        recovery_subtree = py_trees.composites.Selector("Recovery Actions", memory=True)
        recovery_subtree.add_children([
            clear_local_costmap, spin_in_circles  # Recovery actions for obstacle or stuck state
        ])

        # Selector for recovery during patrol
        patrol_with_recovery = py_trees.composites.Sequence("Patrol with Recovery", memory=True)
        patrol_with_recovery.add_children([patrol_route, recovery_subtree])

        # Main Tree: First run the localization init, then the patrol route with recovery handling
        self.root = py_trees.composites.Sequence(name="Main Navigation Tree", memory=True)
        self.root.add_children([init_localization, patrol_with_recovery])

        # BT Creation
        self.tree = py_trees_ros.trees.BehaviourTree(self.root)
        self.get_logger().info("Behavior Tree Initialized")

        # Visualize Tree (optional)
        py_trees.display.render_dot_tree(self.tree.root)

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
    behavior_tree_node = ComplexBehaviorTree()

    try:
        behavior_tree_node.run_tree()
    except KeyboardInterrupt:
        pass
    finally:
        behavior_tree_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
