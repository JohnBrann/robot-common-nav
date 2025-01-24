import rclpy 
import py_trees
import py_trees_ros

import matplotlib.pyplot as plt
import os


class PlotGraph(py_trees.behaviour.Behaviour):
    def __init__(self, node, name):
        """
        Initialize the condition node with a reference to the ROS 2 node
        
        Args:
            name (str): Name of the behavior
            node (rclpy.node.Node): ROS 2 node to access parameters
        """
        super().__init__(name)
        self.node = node
        

    def setup(self):
        """
          What to do here?
          Delayed one-time initialisation that would otherwise interfere
            with offline rendering of this behaviour in a tree to dot graph
            or validation of the behaviour's configuration.
        """
        
    def initialise(self):
        """
        This is called the first time the behaviour is ticked and anytime the status is not RUNNING thereafter.
        """

    def update(self):
        """
        Save/plot rewards graph
        
        Returns:
            py_trees.common.Status: SUCCESS if training is enabled, FAILURE otherwise
        """
        self.plot_episode_rewards()
        
        return py_trees.common.Status.SUCCESS
        
    def terminate(self, new_status):
        """
        This is called when the behaviour switches to a non-running state.
            SUCCESS || FAILURE || INVALID
        """

    def plot_episode_rewards(self):
        """
        Plots the reward per episode and epsilon decay with two y-axes.
        """
        if len(self.node.episode_rewards) == 0:
            self.get_logger().warn("No episode rewards recorded.")
            return
        
        if len(self.node.epsilon_list) == 0:
            self.get_logger().warn("No epsilon values recorded.")
            return
        
        GRAPH_FILE = os.path.join("src", 'rewards_graph_with_epsilon.png')

        # Generate the x-axis for episodes
        episodes = list(range(1, len(self.node.episode_rewards) + 1))
        epsilon_episodes = list(range(1, len(self.node.epsilon_list) + 1))

        # Create the figure and axis
        fig, ax1 = plt.subplots(figsize=(10, 6))

        # Plot rewards on the primary y-axis
        ax1.plot(episodes, self.node.episode_rewards, label='Episode Reward', color='blue')
        ax1.set_xlabel('Episodes')
        ax1.set_ylabel('Mean Reward', color='blue')
        ax1.tick_params(axis='y', labelcolor='blue')
        ax1.grid(True)

        # Create a secondary y-axis for epsilon decay
        ax2 = ax1.twinx()
        ax2.plot(epsilon_episodes, self.node.epsilon_list, label='Epsilon Decay', color='red', linestyle='--')
        ax2.set_ylabel('Epsilon Decay', color='red')
        ax2.tick_params(axis='y', labelcolor='red')

        # Add title and legends
        fig.suptitle('Episode Rewards and Epsilon Decay Over Time')
        ax1.legend(loc='upper left')
        ax2.legend(loc='upper right')

        # Save the graph to file
        plt.savefig(GRAPH_FILE)
        plt.close()

