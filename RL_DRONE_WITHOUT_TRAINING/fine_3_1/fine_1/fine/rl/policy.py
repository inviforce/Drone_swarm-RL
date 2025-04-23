import torch
import torch.nn as nn
import torch.nn.functional as F

def fanin_init(size, fanin=None):
    fanin = fanin or size[0]
    bound = 1. / (fanin ** 0.5)
    return torch.Tensor(size).uniform_(-bound, bound)

class Actor(nn.Module):
    def __init__(self, state_dim, action_dim):
        super(Actor, self).__init__()
        # Medium architecture: smaller hidden layer sizes
        self.fc1 = nn.Linear(state_dim, 256)
        self.ln1 = nn.LayerNorm(256)
        self.fc2 = nn.Linear(256, 128)
        self.ln2 = nn.LayerNorm(128)
        self.fc3 = nn.Linear(128, action_dim)
        self.tanh = nn.Tanh()

        self.init_weights()

    def init_weights(self):
        # Initialize the first two layers with fan-in initialization
        self.fc1.weight.data = fanin_init(self.fc1.weight.data.size())
        self.fc2.weight.data = fanin_init(self.fc2.weight.data.size())
        # Small initialization for the final layer to keep outputs near zero initially
        self.fc3.weight.data.uniform_(-3e-3, 3e-3)

    def forward(self, state):
        x = F.relu(self.ln1(self.fc1(state)))
        x = F.relu(self.ln2(self.fc2(x)))
        return self.tanh(self.fc3(x))

class Critic(nn.Module):
    def __init__(self, state_dim, action_dim):
        super(Critic, self).__init__()
        # First layer processes only the state, with medium dimensions
        self.fc1 = nn.Linear(state_dim, 256)
        self.ln1 = nn.LayerNorm(256)
        # Second layer processes concatenated state and action information
        self.fc2 = nn.Linear(256 + action_dim, 128)
        self.ln2 = nn.LayerNorm(128)
        self.fc3 = nn.Linear(128, 1)

        self.init_weights()

    def init_weights(self):
        self.fc1.weight.data = fanin_init(self.fc1.weight.data.size())
        self.fc2.weight.data = fanin_init(self.fc2.weight.data.size())
        self.fc3.weight.data.uniform_(-3e-3, 3e-3)

    def forward(self, state, action):
        xs = F.relu(self.ln1(self.fc1(state)))
        x = torch.cat([xs, action], dim=1)
        x = F.relu(self.ln2(self.fc2(x)))
        return self.fc3(x)

def soft_update(target, source, tau):
    """Perform a soft update of the target network parameters.
    
    Args:
        target (nn.Module): Target network.
        source (nn.Module): Source network (current network).
        tau (float): Interpolation parameter.
    """
    for target_param, param in zip(target.parameters(), source.parameters()):
        target_param.data.copy_(tau * param.data + (1.0 - tau) * target_param.data)
