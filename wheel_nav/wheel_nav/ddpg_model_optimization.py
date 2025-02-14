import rclpy 
import py_trees
import py_trees_ros
import torch
import torch.nn.functional as F

class DDPGModelOptimization(py_trees.behaviour.Behaviour):
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
    
    def update(self):
        """
        Optimizes DDPG networks
        
        Returns:
            py_trees.common.Status: SUCCESS if training is enabled, FAILURE otherwise
        """

        mini_batch = self.node.memory.sample(self.node.mini_batch_size)

        # Transpose the list of experiences and separate each element
        states, actions, new_states, rewards, terminations = zip(*mini_batch)

        # Prepare tensors for the batch
        states = torch.stack(states).squeeze(1).to(self.device)  # Shape: [batch_size, state_dim]
        new_states = torch.stack(new_states).squeeze(1).to(self.device)  # Shape: [batch_size, state_dim]
        actions = torch.stack(actions).squeeze(1).to(self.device)
        # actions = torch.tensor(actions, dtype=torch.float, device=self.device)
        rewards = torch.tensor(rewards, dtype=torch.float, device=self.device)
        terminations = torch.tensor(terminations, dtype=torch.float, device=self.device)

        # print(f'\nstates: {states}\n\n')

       
        with torch.no_grad():
            # next_action = self.node.actor_target(new_states).clamp(-self.node.action_high, self.node.action_high)
            next_action = self.node.actor_target(new_states)
            target_Q = self.node.critic_target(new_states, next_action)
            # print(f"Target Q after critic target: {target_Q.shape}")
            # target_Q = rewards + (1 - terminations) * self.node.discount_factor_g * target_Q
            target_Q = rewards.unsqueeze(1) + (1 - terminations.unsqueeze(1)) * self.node.discount_factor_g * target_Q.detach()


        # Compute current Q estimates
        current_Q = self.node.critic(states, actions)


        # Ensure target_Q is squeezed to match the shape of current_Q
        # target_Q = target_Q.unsqueeze(1)

        # print(f"current_Q shape: {current_Q.shape}")
        # print(f"target_Q shape: {target_Q.shape}")
        # Compute critic loss
        critic_loss = F.mse_loss(current_Q, target_Q)
        
        # Optimize the critic
        self.node.critic_optimizer.zero_grad()
        critic_loss.backward()
        self.node.critic_optimizer.step()

        # Optimize the actor
        actor_loss = -self.node.critic(states, self.node.actor(states)).mean()

        self.node.actor_optimizer.zero_grad()
        actor_loss.backward()
        self.node.actor_optimizer.step()

        # Update target networks # TEST THISSSSS
        with torch.no_grad():
            for param, target_param in zip(self.node.critic.parameters(), self.node.critic_target.parameters()):
                target_param.data.copy_(self.node.tau * param.data + (1 - self.node.tau) * target_param.data)

            for param, target_param in zip(self.node.actor.parameters(), self.node.actor_target.parameters()):
                target_param.data.copy_(self.node.tau * param.data + (1 - self.node.tau) * target_param.data)

        return py_trees.common.Status.SUCCESS
        

    def terminate(self, new_status):
        """
        This is called when the behaviour switches to a non-running state.
            SUCCESS || FAILURE || INVALID
        """

