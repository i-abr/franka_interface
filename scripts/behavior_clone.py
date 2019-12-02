import pickle as pkl
import torch
import torch.optim as optim
import numpy as np
import argparse
from policy import BasicPolicy

if __name__ == '__main__':

    input_data = []
    output_data = []

    data = pkl.load(open('./data/expert_demo/demo.pkl'))

    input_data = data['inputs']
    output_data = data['outputs']

    input_data = torch.FloatTensor(np.stack(input_data))
    output_data = torch.FloatTensor(np.stack(output_data))

    policy = BasicPolicy(input_data.shape[1], output_data.shape[1])

    optimizer = optim.Adam(policy.parameters(), lr=3e-3)



    for i in range(500):

        dist = policy(input_data)
        loss = - dist.log_prob(output_data).mean()

        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

        if i % 100 == 0:
            print(i, loss.item())

    torch.save(policy.state_dict(), './data/expert_demo/policy_param.pt')
    print('saved behavior cloned policy')
