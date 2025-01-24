import functools
import os
import rclpy
from rclpy.node import Node
import py_trees
import py_trees_ros
import py_trees.display
import torch
from torch import nn
import matplotlib.pyplot as plt

from py_trees.visitors import SnapshotVisitor
from py_trees.display import unicode_tree

from wheel_nav.dqn import DQN
from wheel_nav.experience_replay import ReplayMemory
from wheel_nav.testing_state import TestingModeState
from wheel_nav.training_state import TrainingModeState
from wheel_nav.get_state_data import GetStateData
from wheel_nav.select_discrete_action import SelectDiscreteAction
from wheel_nav.select_continuous_action import SelectContinuousAction
from wheel_nav.calculate_reward import CalcReward
from wheel_nav.append_memory import AppendMemory
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


class RlBehaviorTree(Node):
    def __init__(self):
        super().__init__('rl_bt_node')

        # === Variables for tracking episodes ===
        self.current_episode_reward = 0
        self.current_episode = 0
        self.best_episode_reward = -3000
        self.episode_rewards = []
        self.step_count = 0
        self.epsilon_list = []

        self.declare_parameter('max_steps', 150)
        self.max_steps = self.get_parameter('max_steps').get_parameter_value().integer_value 
        
        # === Model/Graph ===
        self.MODEL_FILE = os.path.join("src", f'dqn.pt')

        # === Exploration/Exploitation settings === 
        self.declare_parameter('epsilon', 0.5)
        self.epsilon = self.get_parameter('epsilon').get_parameter_value().double_value

        # === RL Model Parameters === 
        self.declare_parameter('fc1_nodes', 64)
        self.declare_parameter('num_states', 11)
        self.declare_parameter('num_actions', 10)
        self.declare_parameter('learning_rate_a', 0.003)
        self.declare_parameter('discount_factor_g', 0.98)

        self.num_states = self.get_parameter('num_states').get_parameter_value().integer_value
        self.num_actions = self.get_parameter('num_actions').get_parameter_value().integer_value
        self.fc1_nodes = self.get_parameter('fc1_nodes').get_parameter_value().integer_value
        self.learning_rate_a = self.get_parameter('learning_rate_a').get_parameter_value().double_value
        self.discount_factor_g = self.get_parameter('discount_factor_g').get_parameter_value().double_value


        # === Replay Memory === 
        self.declare_parameter('replay_memory_size', 10000)
        self.declare_parameter('mini_batch_size', 64)
        self.replay_memory_size = self.get_parameter('replay_memory_size').get_parameter_value().integer_value
        self.mini_batch_size =  self.get_parameter('mini_batch_size').get_parameter_value().integer_value

        self.memory = ReplayMemory(self.replay_memory_size)

        # === Models ===
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        self.policy_dqn = DQN(self.num_states, self.num_actions, self.fc1_nodes).to(self.device)
        self.target_dqn = DQN(self.num_states, self.num_actions, self.fc1_nodes).to(self.device)
        self.target_dqn.load_state_dict(self.policy_dqn.state_dict())
        
        self.optimizer = torch.optim.Adam(self.policy_dqn.parameters(), lr=self.learning_rate_a)
        self.loss_fn = nn.MSELoss()

        # === Flags ===
        self.declare_parameter('is_discrete', True)
        self.declare_parameter('is_training', True)
        self.is_discrete = self.get_parameter('is_discrete').get_parameter_value().bool_value
        self.is_training = self.get_parameter('is_training').get_parameter_value().bool_value

        self.training_started = False
        self.allow_optimization = False

        # === Environment State Data ===
        self.distance_to_goal = None
        self.angle_to_goal = None
        self.scan_data = []
        self.min_obstacle_distance = None
        self.angular_velocity = None
        self.linear_velocity = None
        self.action = None
        self.state = None
        self.new_state = None
        self.reward = 0


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

        # === Training Sequence ===
        training_mode_state = TrainingModeState(self, "training?", self.is_training)

        training_started_state = TrainingStartedState(self, "training_started?")
        set_init_training_state = SetInitialTrainingState(self, "set_init_training_state")

        training_started_seq = py_trees.composites.Sequence("Training Started Sequence", memory=True)
        training_started_seq.add_children([training_started_state])
        training_init_seq = py_trees.composites.Sequence("Training Init Sequence", memory=True)
        training_init_seq.add_children([set_init_training_state])

        training_started_sel = py_trees.composites.Selector("Training Started",memory=True)
        training_started_sel.add_children([training_started_seq, training_init_seq])


        # === Select Action Type (Discrete or Continuous) ===
        discrete_action_state = DiscreteActionState(self, "discrete_action?", self.is_discrete)  
        select_discrete_action = SelectDiscreteAction(self, "select_discrete_action")
        discrete_seq = py_trees.composites.Sequence("Discrete Action Sequence", memory=True)
        discrete_seq.add_children([discrete_action_state, select_discrete_action])

        continuous_action_state = ContinuousActionState(self, "continuous_action?", self.is_discrete) 
        select_continuous_action = SelectContinuousAction(self, "select_continuous_action")
        continuous_seq = py_trees.composites.Sequence("Continuous Action Sequence", memory=True)
        continuous_seq.add_children([continuous_action_state, select_continuous_action])

        action_type_sel = py_trees.composites.Selector("Action Type ", memory=True)
        action_type_sel.add_children([discrete_seq, continuous_seq])


        # === Get New State Data after action has been taken ===
        get_state_data = GetStateData(self, "get_state_data")
        state_data_seq =  py_trees.composites.Sequence("State Data Sequence",  memory=True)
        state_data_seq.add_children([get_state_data])


        # === Reward, Memory, State update ===
        get_reward = CalcReward(self, "get_reward")
        append_memory = AppendMemory(self, "append_memory")
        new_state_update = UpdateState(self, "new_state_update")

        data_seq = py_trees.composites.Sequence("Reward/Memory/New State Sequence", memory= True)
        data_seq.add_children([get_reward, append_memory, new_state_update])

        # === End of Episode Check ===
        plot_graph_on_failure = PlotGraph(self, "PlotGraph")

        # Success State
        success_state = EpisodeSuccessState(self, "episode_success_state?")
        reset_env_on_success = ResetEnvSuccess(self, "reset_env_on_success")
        success_state_seq = py_trees.composites.Sequence("Success State Sequence", memory=True)
        success_state_seq.add_children([success_state, reset_env_on_success])

        # Failure State
        terminated_state = EpisodeFailureState(self, "episode_failure_state?")
        reset_env_on_failure = ResetEnvFailure(self,"reset_env_on_failure")
        terminated_state_seq = py_trees.composites.Sequence("Failure State Sequence", memory=True)
        terminated_state_seq.add_children([terminated_state, reset_env_on_failure, plot_graph_on_failure])

        # Running State (Not Success or Failure)
        runnning_state = EpisodeRunningState(self, "episode_running_state?")
        running_state_seq = py_trees.composites.Sequence("Running State Sequence", memory=True)
        running_state_seq.add_children([runnning_state])

        episode_state = py_trees.composites.Selector("Episode State", memory=True)
        episode_state.add_children([success_state_seq, terminated_state_seq, running_state_seq])


        # === Model Optimization ===
        memory_length_state = MemoryLengthState(self, "memory_length?")
        optimization_step_state = AllowOptimizationState(self, "optimization step?")
        dqn_optimization = DQNModelOptimization(self, "DQN Model Optimization", self.device)

        optimization_sel = py_trees.composites.Selector("Optimization", memory=True)
        optimization_sel.add_children([memory_length_state, optimization_step_state, dqn_optimization])


        # === Step Delay ===
        step_delay = StepTimer(self, "step_delay")
        step_delay_seq = py_trees.composites.Sequence("Step Delay Sequence", memory=True)
        step_delay_seq.add_children([step_delay])


        # === Training Sequence ===
        training_seq = py_trees.composites.Sequence("Training Sequence", memory=True)
        training_seq.add_children([training_mode_state, training_started_sel, action_type_sel, state_data_seq, data_seq, episode_state, optimization_sel, step_delay_seq]) 

        # === Testing Sequence === 
        testing_mode_state = TestingModeState(self, "testing?", self.is_training)

        testing_seq = py_trees.composites.Sequence("Testing Sequence", memory=True)
        testing_seq.add_children([testing_mode_state])

        mode_sel = py_trees.composites.Selector("Mode. Training or Testing", memory=True)
        mode_sel.add_children([training_seq, testing_seq])

        # === Root === 
        self.root = py_trees.composites.Sequence(name="Root", memory=True)
        self.root.add_children([mode_sel])

        # === BT Creation ===
        self.tree = py_trees_ros.trees.BehaviourTree(self.root)
        self.get_logger().info("Behavior Tree Initialized")


        # Visualize Tree
        py_trees.display.render_dot_tree(self.tree.root)


    def run_tree(self):
        try:
            while rclpy.ok():
                # Tick the behavior tree
                self.tree.tick()

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