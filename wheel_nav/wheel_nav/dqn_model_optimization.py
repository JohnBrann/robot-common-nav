import rclpy 
import py_trees
import py_trees_ros
import torch

class DQNModelOptimization(py_trees.behaviour.Behaviour):
    def __init__(self, node, name, device):
        """
        Initialize the condition node with a reference to the ROS 2 node
        
        Args:
            name (str): Name of the behavior
            node (rclpy.node.Node): ROS 2 node to access parameters
        """
        super().__init__(name)
        self.node = node
        self.device = device
        

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
        self.training_started = self.node.training_started
    

    def update(self):
        """
        Optimizes DQN networks
        
        Returns:
            py_trees.common.Status: SUCCESS if training is enabled, FAILURE otherwise
        """

        mini_batch = self.node.memory.sample(self.node.mini_batch_size)

        # Transpose the list of experiences and separate each element
        states, actions, new_states, rewards, terminations = zip(*mini_batch)

        # Prepare tensors for the batch
        states = torch.stack(states).squeeze(1)  # Shape: [batch_size, state_dim]
        new_states = torch.stack(new_states).squeeze(1)  # Shape: [batch_size, state_dim]
        actions = torch.tensor(actions, dtype=torch.float, device=self.device)
        rewards = torch.tensor(rewards, dtype=torch.float, device=self.device)
        terminations = torch.tensor(terminations, dtype=torch.float, device=self.device)

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

