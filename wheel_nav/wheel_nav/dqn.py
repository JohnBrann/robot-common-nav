import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F

class DQN(nn.Module):
    def __init__(self, state_size, action_size=5, hidden_size=256):
        """
        Initialize the Deep Q-Network.

        Args:
            state_size (int): Dimension of the input state space.
            action_size (int): Number of possible actions.
            hidden_size (int): Number of units in the hidden layers.
        """
        super(DQN, self).__init__()
        self.fc1 = nn.Linear(state_size, hidden_size)
        self.fc2 = nn.Linear(hidden_size, hidden_size)
        self.output = nn.Linear(hidden_size, action_size)

    def forward(self, x):
        """
        Perform a forward pass through the network.

        Args:
            x (torch.Tensor): Input state.

        Returns:
            torch.Tensor: Q-values for each action.
        """
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        return self.output(x)

