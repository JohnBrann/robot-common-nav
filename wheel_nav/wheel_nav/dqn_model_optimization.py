import rclpy 
import py_trees
import py_trees_ros
import torch

class DQNModelOptimization(py_trees.behaviour.Behaviour):
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
        self.training_started = self.node.training_started
        # self.node.get_logger().info(f"Determining Training Mode... is_training?")
        

    def update(self):
        """
        Single check to see if training has started
        
        Returns:
            py_trees.common.Status: SUCCESS if training is enabled, FAILURE otherwise
        """




        mini_batch = self.node.memory.sample(self.node.mini_batch_size)

        # Transpose the list of experiences and separate each element
        states, actions, new_states, rewards, terminations = zip(*mini_batch)

        # Stack tensors to create batch tensors
        # tensor([[1,2,3]])
        # Stack tensors to create batch tensors
        states = torch.stack(states)  # Already tensors from the state collection
        states = states.squeeze(dim=1)
        new_states = torch.stack(new_states)  # Already tensors from the state collection
        new_states = new_states.squeeze(dim=1)  # Shape: [100, 17]

        # Convert actions, rewards, and terminations to tensors
        actions = torch.tensor(actions, dtype=torch.long, device=self.node.device)
        rewards = torch.tensor(rewards, dtype=torch.float, device=self.node.device)
        terminations = torch.tensor(terminations, dtype=torch.float, device=self.node.device)

        # self.node.get_logger().info(f"Actions shape before unsqueeze: {actions.shape}")
        # self.node.get_logger().info(f"Actions shape after unsqueeze: {actions.unsqueeze(dim=1).shape}")
        # # self.node.get_logger().info(f"Actions values: {actions}")



        # self.node.get_logger().info(f"rewards shape: {rewards.shape}")
        # self.node.get_logger().info(f"terminations shape: {terminations.shape}")
        # self.node.get_logger().info(f"new_states shape: {new_states.shape}")
        # self.node.get_logger().info(f"states shape: {states.shape}")
        # self.node.get_logger().info(f"target_dqn output shape: {self.node.target_dqn(new_states).shape}")
        # self.node.get_logger().info(f"policy_dqn output shape: {self.node.policy_dqn(states).shape}")




        with torch.no_grad():
            # Calculate target Q values (expected returns)
            target_q = rewards + (1-terminations) * self.node.discount_factor_g * self.node.target_dqn(new_states).max(dim=1)[0]
            '''
                target_dqn(new_states)  ==> tensor([[1,2,3],[4,5,6]])
                    .max(dim=1)         ==> torch.return_types.max(values=tensor([3,6]), indices=tensor([3, 0, 0, 1]))
                        [0]             ==> tensor([3,6])
            '''


         # Calcuate Q values from current policy
        current_q = self.node.policy_dqn(states).gather(dim=1, index=actions.unsqueeze(dim=1)).squeeze()
        '''
            policy_dqn(states)  ==> tensor([[1,2,3],[4,5,6]])
                actions.unsqueeze(dim=1)
                .gather(1, actions.unsqueeze(dim=1))  ==>
                    .squeeze()                    ==>
        '''


        # Compute loss
        loss = self.node.loss_fn(current_q, target_q)

        # Optimize the model (backpropagation)
        self.node.optimizer.zero_grad()  # Clear gradients
        loss.backward()             # Compute gradients
        self.node.optimizer.step()       # Update network parameters i.e. weights and biases


        
        return py_trees.common.Status.SUCCESS
        
        
    def terminate(self, new_status):
        """
        This is called when the behaviour switches to a non-running state.
            SUCCESS || FAILURE || INVALID
        """
        # self.node.get_logger().info(f"Terminating TrainingModeState with status {new_status}")

