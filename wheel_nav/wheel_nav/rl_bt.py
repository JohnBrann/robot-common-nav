import rclpy
from rclpy.node import Node
import py_trees
import py_trees_ros
import py_trees.display

from wheel_nav.experience_replay import ReplayMemory

from wheel_nav.testing_state import TestingModeState
from wheel_nav.training_state import TrainingModeState

from wheel_nav.select_discrete_action import SelectDiscreteAction
from wheel_nav.select_continuous_action import SelectContinuousAction
from wheel_nav.calculate_reward import CalcReward
from wheel_nav.append_memory import AppendMemory
# from wheel_nav.optimize_model import OptimizeModel

from wheel_nav.discrete_action_state import DiscreteActionState
from wheel_nav.continuous_action_state import ContinuousActionState
from wheel_nav.reset_env_success import ResetEnvSuccess
from wheel_nav.reset_env_failure import ResetEnvFailure

from wheel_nav.episode_success_state import EpisodeSuccessState
from wheel_nav.episode_failure_state import EpisodeFailureState
from wheel_nav.episode_running_state import EpisodeRunningState



class RlBehaviorTree(Node):
    def __init__(self):
        super().__init__('rl_bt_node')
        

        # Would load hyperparamters, but for now we have these:

        self.current_episode_reward = 0
        self.current_episode = 0
        self.episode_rewards = []
        self.step_count = 0
        self.max_steps = 100

        # Replay Memory, is the initializeing process okay to be here?
        self.replay_memory_size = 10000
        self.memory = ReplayMemory(self.replay_memory_size)

        # Network creation


        self.is_discrete = True
        self.is_training = True
        # Create a behavior tree
        self.create_behavior_tree()
        
    def create_behavior_tree(self):

        # Reset Env
        reset_env_success = ResetEnvSuccess(self, "reset_env_on_success")

        reset_env_failure = ResetEnvFailure(self,"reset_env_on_failure")

        # Success State
        success_state = EpisodeSuccessState(self, "episode_success_state")
        success_state_sequence = py_trees.composites.Sequence("Success State Sequence", memory=True)
        success_state_sequence.add_children([success_state, reset_env_success])

        # Terminated State 
        terminated_state = EpisodeFailureState(self, "episode_failure_state")
        terminated_state_sequence = py_trees.composites.Sequence("Failure State Sequence", memory=True)
        terminated_state_sequence.add_children([terminated_state, reset_env_failure])

        # In Progress State (Not terminated or Success)
        runnning_state = EpisodeRunningState(self, "episode_running_state")
        running_state_sequence = py_trees.composites.Sequence("Running State Sequence", memory=True)
        running_state_sequence.add_children([runnning_state])

        episode_state = py_trees.composites.Selector("Episode State", memory=True)
        episode_state.add_children([success_state_sequence, terminated_state_sequence, running_state_sequence])


        # Training Behaviors
        select_action_discrete = SelectDiscreteAction(self, "select_discrete_action")
        select_action_continuous = SelectContinuousAction(self, "select_continuous_action")
        reward_calc = CalcReward(self, "calc_reward")
        append_memory = AppendMemory(self, "append_memory")

        # Action Type (Discrete or Continuous)
        discrete_action = DiscreteActionState(self, "discrete_action?", self.is_discrete)  
        continuous_action = ContinuousActionState(self, "continuous_action?", self.is_discrete) 

        da_sequence = py_trees.composites.Sequence("Discrete Action Sequence", memory=True)
        da_sequence.add_children([discrete_action, select_action_discrete])
        ca_sequence = py_trees.composites.Sequence("Continuous Action Sequence", memory=True)
        ca_sequence.add_children([continuous_action, select_action_continuous])

        # Epsiode Handling Nodes
        # episode_state = EpisodeState(self, "episode_state") # use a ros2 parameter to keep track of which episode we are on
        # memory_buffer = UpdateMemoryBuffer(self, "update_memory_buffer") # where is this held? ros2 param?

        # Action Type Selector (discrete or continuous)
        action_type = py_trees.composites.Selector("Action Type", memory=True)
        action_type.add_children([da_sequence, ca_sequence])

        data_sequence = py_trees.composites.Sequence("Data/Reward Sequence", memory= True)
        # data_sequence.add_children([episode_state, reward_calc, memory_buffer])
        data_sequence.add_children([reward_calc, append_memory])

        optimization_sequence = py_trees.composites.Sequence("Model Optimization", memory=True)

        # Training Sequence (sequentially execute action and reward calculation)
        training_check = TrainingModeState(self, "training?", self.is_training)
        training_seq = py_trees.composites.Sequence("Training Sequence", memory=True)
        training_seq.add_children([training_check, action_type, data_sequence, episode_state, optimization_sequence]) 

        # Testing Sequence 
        testing_check = TestingModeState(self, "testing?", self.is_training)

        testing_seq = py_trees.composites.Sequence("Testing Sequence", memory=True)
        testing_seq.add_children([testing_check])

        # end_of_episodes_sequence = py_trees.composites("End of Epsiode Handing", memory=True)
        # end_of_episodes_sequence.add_children([episode_state])

        mode_type = py_trees.composites.Selector("Mode. Training or Testing", memory=True)
        mode_type.add_children([training_seq, testing_seq])
        # Root (start from the training sequence)
        self.root = py_trees.composites.Sequence(name="Root", memory=True)
        self.root.add_children([mode_type])

        # BT Creation
        self.tree = py_trees_ros.trees.BehaviourTree(self.root)
        self.get_logger().info("Behavior Tree Initialized")


        # Visualize Tree
        py_trees.display.render_dot_tree(self.tree.root)


    def run_tree(self):
        try:
            while rclpy.ok():
                # Tick the behavior tree
                self.tree.tick()

                self.get_logger().info(f"Total Steps: {self.step_count}")

                # Allow time for ROS2 to process messages
                rclpy.spin_once(self, timeout_sec=0.1)
        except KeyboardInterrupt:
            self.get_logger().info("Behavior tree execution interrupted by user.")


    # Functions called from the behaviors

    def add_reward_to_episode(self, reward): 
        """
        Add reward to the current episode's total reward.
        """
        self.current_episode_reward += reward
    
    def add_to_step_count(self): 
        """
        Add reward to the current episode's total reward.
        """
        self.step_count += 1

    def reset_step_count(self):
        """
        Reset the step count to 0
        """
        self.step_count = 0

    def episode_truncated(self):
        return self.step_count > self.max_steps



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
