import torch
import torch.nn.functional as F
import torch.nn as nn
import torch.optim as optim
import numpy as np

class Actor(nn.Module):
    def __init__(self, name, state_size, action_size, hidden_size):
        super(Actor, self).__init__(name)
        # --- define layers here ---
        self.fa1 = nn.Linear(state_size, hidden_size)
        self.fa2 = nn.Linear(hidden_size, hidden_size)
        self.fa3 = nn.Linear(hidden_size, action_size)
        # --- define layers until here ---

        self.apply(super().init_weights)

    def forward(self, states):
        # --- define forward pass here ---
        x1 = torch.relu(self.fa1(states))
        x2 = torch.relu(self.fa2(x1))
        action = torch.tanh(self.fa3(x2))

        return action

class Critic(nn.Module):
    def __init__(self, name, state_size, action_size, hidden_size):
        super(Critic, self).__init__(name)

        # --- define layers here ---
        self.l1 = nn.Linear(state_size, int(hidden_size / 2))
        self.l2 = nn.Linear(action_size, int(hidden_size / 2))
        self.l3 = nn.Linear(hidden_size, hidden_size)
        self.l4 = nn.Linear(hidden_size, 1)
        # --- define layers until here ---

        self.apply(super().init_weights)

    def forward(self, states, actions):
        # --- define forward pass here ---
        xs = torch.relu(self.l1(states))
        xa = torch.relu(self.l2(actions))
        x = torch.cat((xs, xa), dim=1)
        x = torch.relu(self.l3(x))
        x = self.l4(x)
        return x
