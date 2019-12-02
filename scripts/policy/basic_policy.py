import torch
from torch.distributions import Normal
import torch.nn.functional as F
import torch.nn as nn

class Policy(nn.Module):

    def __init__(self, num_inputs, num_outputs, n_hidden=200):

        super(Policy, self).__init__()

        self.linear1        = nn.Linear(num_inputs, n_hidden)
        self.linear2        = nn.Linear(n_hidden, n_hidden)
        self.linear_mean    = nn.Linear(n_hidden, num_outputs)

        self.linear_sigma   = nn.Linear(n_hidden, num_outputs)

    def forward(self, state):
        x = state
        z = F.relu(self.linear1(x))
        x = torch.tanh(self.linear_mean(F.relu(self.linear2(z))))
        sigma = torch.clamp(self.linear_sigma(z), -20., 4.).exp()

        return Normal(x, sigma)

    def get_action(self, state):
        dist = self.forward(torch.FloatTensor(state).unsqueeze(0))
        return dist.sample()[0].data.numpy()
