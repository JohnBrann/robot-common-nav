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

        # self.node.get_logger().info(f"Setting up TrainingModeState with is_training = {self.is_training}")
        

    def initialise(self):
        """
        This is called the first time the behaviour is ticked and anytime the status is not RUNNING thereafter.
        """
        # self.node.get_logger().info(f"Determining Training Mode... is_training?")
        

    def update(self):
        """
        Check the 'is_training' parameter
        
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
        # self.node.get_logger().info(f"Terminating TrainingModeState with status {new_status}")

    def plot_episode_rewards(self):
        """
        Plots the reward per episode.
        """
        if len(self.node.episode_rewards) == 0:
            self.get_logger().warn("No episode rewards recorded.")
            return
        
        GRAPH_FILE = os.path.join("src", f'rewards_graph.png')

        episodes = list(range(1, len(self.node.episode_rewards) + 1))

        plt.figure(figsize=(10, 6))
        plt.plot(episodes, self.node.episode_rewards, label='Episode Reward', marker='o')
        plt.xlabel('Episode')
        plt.ylabel('Cumulative Reward')
        plt.title('Episode Rewards Over Time')
        plt.grid(True)
        plt.legend()
        plt.savefig(GRAPH_FILE)
        plt.close()

