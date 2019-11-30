#!/usr/bin/env python3


import numpy as np

import time


from franka_pickNplace import FrankaPickNPlace
from multmodel import get_multmodel
# from minitaur_multmodel import MinitaurMultModel
from scipy.signal import savgol_filter

import matplotlib.pyplot as plt

def mppi(state, model, u_seq, horizon, lam=0.01, sig=0.1):
    assert len(u_seq) == horizon

    model.set_state(state)

    s   = []
    eps = []
    for t in range(horizon):
        eps.append(np.random.normal(0., sig, size=(model.n_sims, model.action_space.shape[0])))
        obs, rew, done, _ = model.step(u_seq[t] + eps[-1])
        s.append(rew)

    s = np.cumsum(s[::-1], 0)[::-1, :]

    for t in range(horizon):
        s[t] -= np.min(s[t])
        w = np.exp(-s[t]/lam) + 1e-8 # offset
        w /= np.sum(w)
        u_seq[t] = u_seq[t] + np.dot(w, eps[t])
    return savgol_filter(u_seq, horizon-1, 3, axis=0)

env = FrankaPickNPlace(viewer=False)
Model = get_multmodel(FrankaPickNPlace)
model = Model(n_sims=10)


env.reset()
model.reset()

horizon = 10

num_actions = env.action_space.shape[0]

u_seq = [model.def_action.copy() for _ in range(horizon)]

t = 0
# plt.ion()

while True:

    state = env.get_state()

    start = time.time()

    u_seq = mppi(state, model, u_seq, horizon)

    # obs, rew, done, _ = env.step(u_seq[0]*0 + np.array([0.]*4 + [-1.0]*4) )
    # obs, rew, done, _ = env.step(u_seq[0]*0 + np.sin(np.pi*t/30))
    obs, rew, done, _ = env.step(u_seq[0])

    end = time.time()
    print('elapsed time : ', end - start)

    # env.render()
    u_seq[:-1] = u_seq[1:]
    u_seq[-1]  = np.zeros(env.action_space.shape[0])

    # plt.clf()
    # plt.plot(u_seq)
    # plt.draw()
    # plt.pause(0.001)

    t += 1
