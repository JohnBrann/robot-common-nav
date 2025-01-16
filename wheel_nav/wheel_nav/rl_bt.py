import rclpy
from rclpy.node import Node
import py_trees
import py_trees_ros
import py_trees.display
import torch
from torch import nn

from wheel_nav.dqn import DQN

import matplotlib.pyplot as plt


import functools
from py_trees.visitors import SnapshotVisitor
from py_trees.display import unicode_tree

from wheel_nav.experience_replay import ReplayMemory

from wheel_nav.testing_state import TestingModeState
from wheel_nav.training_state import TrainingModeState

from wheel_nav.get_state_data import GetStateData

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

from wheel_nav.step_timer import StepTimer

from wheel_nav.set_initial_training_state import SetInitialTrainingState
from wheel_nav.training_started_state import TrainingStartedState
from wheel_nav.update_state import UpdateState

from wheel_nav.dqn_model_optimization import DQNModelOptimization
from wheel_nav.memory_length_state import MemoryLengthState
from wheel_nav.allow_optimization_state import AllowOptimizationState

from wheel_nav.plot_graph import PlotGraph

import os


class RlBehaviorTree(Node):
    def __init__(self):
        super().__init__('rl_bt_node')

        # todo: load ros params for variable below

        self.current_episode_reward = 0
        self.current_episode = 0
        self.best_episode_reward = -3000
        self.episode_rewards = []
        self.step_count = 0
        self.max_steps = 150

        self.epsilon = 0.5

        self.MODEL_FILE = os.path.join("src", f'overnight_dqn_model.pt')

        # This is all state data, how are we going to hold this data across the system
        # previously data was constantly published to a topic, however this causes inconsistencies across the bt
        # Can we make these variables so they are just published to a topic, or does it just make sense to hold them all here??? 
        # for now I am just putting them here. Data can be accessed across tre without an issue, but is it the best way tho???
        self.distance_to_goal = None
        self.angle_to_goal = None
        self.scan_data = []
        self.min_obstacle_distance = None
        self.angular_velocity = None
        self.linear_velocity = None
        
        self.action = None
        self.state = None
        self.new_state = None

        
        # self.terminated = False
            # self.success = False
        self.reward = 0

        self.training_started = False

        self.fc1_nodes = 64

        self.num_states = 9
        self.num_actions = 5

        self.learning_rate_a = 0.003
        self.discount_factor_g = 0.95
        self.loss_fn = nn.MSELoss()

        self.allow_optimization = False


        self.declare_parameter('terminated', False)
        self.declare_parameter('success', False)
        self.declare_parameter('reward', 0.0)

        # ros2 run wheel_nav rl_bt --ros-args --params-file `ros2 pkg prefix wheel_nav_msgs`/config/params.yaml

        
        # Replay Memory, is the initializing process okay to be here?
        self.replay_memory_size = 10000
        self.mini_batch_size = 100
        self.memory = ReplayMemory(self.replay_memory_size)
        
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        # Network creation
        # todo: start here, look at the number of states
        # 17 is number of states (currently 12 lidar scans + 5 other data) 5 is the number of actions (outputs)
        self.policy_dqn = DQN(self.num_states, self.num_actions, self.fc1_nodes).to(self.device)

        self.target_dqn = DQN(self.num_states, self.num_actions, self.fc1_nodes).to(self.device)
        self.target_dqn.load_state_dict(self.policy_dqn.state_dict())

        self.optimizer = torch.optim.Adam(self.policy_dqn.parameters(), lr=self.learning_rate_a)


        self.is_discrete = True
        self.is_training = True
        # Create a behavior tree
        self.create_behavior_tree()

        # # Initialize the SnapshotVisitor
        # self.snapshot_visitor = SnapshotVisitor()

        # # Add the post-tick handler to the tree
        # self.tree.add_post_tick_handler(
        #     functools.partial(self.post_tick_handler, self.snapshot_visitor)
        # )
        # self.tree.visitors.append(self.snapshot_visitor)

        
    def create_behavior_tree(self):

        # Initial state data at the beginning of the training, this happens only once
        set_init_training_state = SetInitialTrainingState(self, "set_init_training_state")
        training_started_state = TrainingStartedState(self, "training_started?")

        training_started = py_trees.composites.Selector("Training Started",memory=True)
        training_started.add_children([training_started_state, set_init_training_state])

        plot_graph_failure = PlotGraph(self, "PlotGraph")



        # get state data at beginning of step to be consistent across the tree step
        get_state_data = GetStateData(self, "get_state_data")
        get_state_data_seq =  py_trees.composites.Sequence("State Data Sequence",  memory=True)
        get_state_data_seq.add_children([get_state_data])

        # Reset Env
        reset_env_success = ResetEnvSuccess(self, "reset_env_on_success")

        reset_env_failure = ResetEnvFailure(self,"reset_env_on_failure")

        # Success State
        success_state = EpisodeSuccessState(self, "episode_success_state?")
        success_state_sequence = py_trees.composites.Sequence("Success State Sequence", memory=True)
        success_state_sequence.add_children([success_state, reset_env_success])

        # Terminated State 
        terminated_state = EpisodeFailureState(self, "episode_failure_state?")
        terminated_state_sequence = py_trees.composites.Sequence("Failure State Sequence", memory=True)
        terminated_state_sequence.add_children([terminated_state, reset_env_failure, plot_graph_failure])

        # In Progress State (Not terminated or Success)
        runnning_state = EpisodeRunningState(self, "episode_running_state?")
        running_state_sequence = py_trees.composites.Sequence("Running State Sequence", memory=True)
        running_state_sequence.add_children([runnning_state])

        episode_state = py_trees.composites.Selector("Episode State", memory=True)
        episode_state.add_children([success_state_sequence, terminated_state_sequence, running_state_sequence])


        # Training Behaviors
        select_action_discrete = SelectDiscreteAction(self, "select_discrete_action")
        select_action_continuous = SelectContinuousAction(self, "select_continuous_action")
        reward_calc = CalcReward(self, "calc_reward")
        append_memory = AppendMemory(self, "append_memory")
        update_state = UpdateState(self, "Update State")

        # Action Type (Discrete or Continuous)
        discrete_action = DiscreteActionState(self, "discrete_action?", self.is_discrete)  
        continuous_action = ContinuousActionState(self, "continuous_action?", self.is_discrete) 

        da_sequence = py_trees.composites.Sequence("Discrete Action Sequence", memory=True)
        da_sequence.add_children([discrete_action, select_action_discrete])
        ca_sequence = py_trees.composites.Sequence("Continuous Action Sequence", memory=True)
        ca_sequence.add_children([continuous_action, select_action_continuous])

        # Action Type Selector (discrete or continuous)
        action_type = py_trees.composites.Selector("Action Type", memory=True)
        action_type.add_children([da_sequence, ca_sequence])


        data_sequence = py_trees.composites.Sequence("Data/Reward Sequence", memory= True)
        # data_sequence.add_children([episode_state, reward_calc, memory_buffer])
        data_sequence.add_children([reward_calc, append_memory, update_state])


        dqn_optimization = DQNModelOptimization(self, "DQN Model Optimization")
        memory_length = MemoryLengthState(self, "memory_length?")
        optimization_step = AllowOptimizationState(self, "Optimzation step?")

        optimization_selector = py_trees.composites.Selector("Optimization", memory=True)
        optimization_selector.add_children([memory_length, optimization_step, dqn_optimization])
        # optimization_sequence = py_trees.composites.Sequence("Model Optimization", memory=True)

        step_delay = StepTimer(self, "step_delay")

        step_delay_seq = py_trees.composites.Sequence("Step Delay Sequence", memory=True)
        step_delay_seq.add_children([step_delay])


        # Training Sequence (sequentially execute action and reward calculation)
        training_check = TrainingModeState(self, "training?", self.is_training)
        training_seq = py_trees.composites.Sequence("Training Sequence", memory=True)
        training_seq.add_children([training_check, training_started, action_type, get_state_data_seq, data_sequence, episode_state, optimization_selector, step_delay_seq]) 

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

    def post_tick_handler(self, snapshot_visitor, behaviour_tree):
        """
        Post-tick handler to display the tree in Unicode format.
        """
        tree_representation = unicode_tree(
            behaviour_tree.root,
            visited=snapshot_visitor.visited,
            previously_visited=snapshot_visitor.visited
        )
        self.get_logger().info("\n" + tree_representation)

    def add_reward_to_episode(self, reward): 
        """
        Add step reward to the current episodes cumulative reward
        """
        self.current_episode_reward += reward

    def reset_episode_reward(self):
        """
        Reset the episode reward
        """
        self.current_episode_reward = 0

    def get_current_episode(self):
        """
        Get the current episode number
        """
        return self.current_episode
    
    def get_current_episode_reward(self):
        """
        Return the cumulative reward of the current episode
        """
        return self.current_episode_reward
    
    def add_to_episode_count(self):
        self.current_episode += 1

    def add_to_step_count(self): 
        """
        Add reward to the current episode's total reward
        """
        self.step_count += 1

    def reset_step_count(self):
        """
        Reset the step count to 0
        """
        self.step_count = 0

    def episode_truncated(self):
        return self.step_count > self.max_steps
    

    # def set_step_data(self, distance_to_goal, angle_to_goal, scan_data, min_obstacle_distance, angular_velocity, linear_velocity, terminated, success, reward):
    #     self.distance_to_goal = distance_to_goal
    #     self.angle_to_goal = angle_to_goal
    #     self.scan_data = scan_data
    #     self.min_obstacle_distance = min_obstacle_distance
    #     self.angular_velocity = angular_velocity
    #     self.linear_velocity = linear_velocity
    #     self.terminated = terminated
    #     self.success = success
    #     self.reward = reward

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
