import numpy as np
from mujoco_py import MjSimPool, MjSim
from gym.spaces import Box

from quatmath import mat2euler, mat2quat


def get_multmodel(Base):

    class MultModel(Base):

        def __init__(self, n_sims=20):

            self.n_sims = n_sims

            Base.__init__(self, viewer=False)

            self.pool = [MjSim(self.model) for _ in range(n_sims)]
            self.pool = MjSimPool(self.pool, nsubsteps=self.frame_skip)

            self.data = [sim.data for sim in self.pool.sims]

        def set_state(self, state):
            for sim in self.pool.sims:
                sim.set_state(state)
            self.pool.forward()


        def reset(self):
            self.pool.reset()
            self.pool.forward()
            obs = np.stack([self.get_obs(data) for data in self.data])
            return obs

        def step(self, a):

            for i, data in enumerate(self.data):
                ctrl = np.clip(a[i], -1.0, 1.0)
                ctrl = self.act_mid + ctrl * self.act_rng
                data.ctrl[:] = ctrl.copy()
            self.pool.step()

            rew, obs = self.get_batch_rew_obs()
            done = False
            return obs, rew, done, {}

        def get_batch_rew_obs(self):
            rew = []
            obs = []
            for data in self.data:
                obs.append(self.get_obs(data))
                rew.append(self.get_rew(data))
            return np.stack(rew), np.stack(obs)

    return MultModel
