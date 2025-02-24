import torch
import torch.nn.functional as F
import torch.nn as nn
import torch.optim as optim
import numpy as np

class Actor(nn.Module):
    def __init__(self, state_size, action_size, max_action, hidden_size):
            super(Actor, self).__init__()
            self.fc1 = nn.Linear(state_size, 256)
            self.fc2 = nn.Linear(256, 256)
            # self.fc3 = nn.Linear(256, 256)
            self.fc4 = nn.Linear(256, 2)  # Two outputs: linear and angular velocity

            self.tanh = nn.Tanh()  # For angular velocity (-1 to 1)
            self.sigmoid = nn.Sigmoid()  # For linear velocity (0 to 1)


    def forward(self, state):
        

        # print(f'state shape in forward: {state.shape}')

         # --- define forward pass here ---
        x1 = torch.relu(self.fc1(state))
        x2 = torch.relu(self.fc2(x1))
        # x3 = torch.relu(self.fc3(x2))
        action = self.fc4(x2)  # [batch_size, 2] (linear_velocity, angular_velocity)

        # print(f'action shape before scaling: {action.size()}')

        # Ensure action has the correct shape [batch_size, 2]
        action = action.view(-1, 2)  # Flatten if necessary
        
        # print(f'linear before: {action[:,0]}')
        linear_velocity = self.sigmoid(action[:, 0]) * 0.22  # Scale to [0, 0.22]
        # print(f'linear: {linear_velocity}')
        
        # print(f'angulare before: {action[:,1]}')
        angular_velocity = self.tanh(action[:, 1]) * 1.0  # Scale to [-1, 1]
        # print(f'angular: {angular_velocity}')

        # Stack them into a single action tensor: [batch_size, 2]
        action = torch.stack([linear_velocity, angular_velocity], dim=1)
        # print(f'action selected: {action}')

        return action

class Critic(nn.Module):
    def __init__(self, state_size, action_size, max_action, hidden_size):
        super(Critic, self).__init__()
        # State and action branches
        self.l1 = nn.Linear(state_size, hidden_size // 2)  # Process state
        self.l2 = nn.Linear(action_size, hidden_size // 2)  # Process action

        # Fully connected layers after concatenation
        self.l3 = nn.Linear(hidden_size, hidden_size * 2)  # More neurons for better learning
        self.l4 = nn.Linear(hidden_size * 2, 1)  # Single output: Q-value

    def forward(self, states, actions):
        xs = torch.relu(self.l1(states))
        xa = torch.relu(self.l2(actions))
        x = torch.cat((xs, xa), dim=1)  # Concatenate processed state & action
        x = torch.relu(self.l3(x))
        return self.l4(x)  # Q-value output
