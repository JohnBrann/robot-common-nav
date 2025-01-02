import rclpy
from rclpy.node import Node
import py_trees
import py_trees_ros
import py_trees.display

from wheel_nav.training_mode_state import TrainingModeState


class RlBehaviorTree(Node):
    def __init__(self):
        super().__init__('rl_bt_node')

        # Declare parameters
        self.declare_parameter('is_training', True)

        # Get parameters
        self.is_training = self.get_parameter('is_training').value

         # Create a behavior tree
        self.create_behavior_tree()
        
    def create_behavior_tree(self):
        # Create a root node and add child nodes to it
        self.root = py_trees.composites.Sequence(name="Root", memory=True)

        # Create custom behavior for training mode
        training_check = TrainingModeState(self, name="TrainingMode", is_training=self.is_training)
        testing_check = TrainingModeState(self, name="TrainingMode", is_training=self.is_training)



        # Start here, we have a check for wether or not we are training or not, whats the next hehavior, think back to rl scripts
        # initilization portion, what else needs to be initialized, initilaize enviroment, start of run, then create episode loop and see if you can jsut get data like with the script agent. 
        # test using black board for counting, can it keep track?
        # how do we actually maintain data, memory, the model thoughtout the whole trainning process. 

        training_branch = py_trees.composites.Sequence(name="Training", memory=True)
        training_branch.add_children([training_check])
        testing_branch = py_trees.composites.Sequence(name="Testing", memory=True)
        testing_branch.add_children([testing_check])


        # Add the behavior to the root
        self.root.add_children([training_branch, testing_branch])

        # Set the tree
        self.tree = py_trees_ros.trees.BehaviourTree(self.root)

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
    behavior_tree_node = RlBehaviorTree()

    try:
        behavior_tree_node.run_tree()
    except KeyboardInterrupt:
        pass
    finally:
        behavior_tree_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
