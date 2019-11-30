import numpy as np
from mujoco_py import load_model_from_path, MjSim, MjViewer
from gym.spaces import Box

from quatmath import mat2euler, mat2quat, euler2mat

TARGET_GRIPPER_CONFIG = mat2quat(np.array([
            [0., 0., 1.],
            [1., 0., 0.],
            [0., 1., 0.]
        ]))

ROTY = euler2mat(np.array([0, np.pi/2, 0]))
ROTX = euler2mat(np.array([np.pi, 0., 0.]))

class FrankaPickNPlace(object):


    def __init__(self, frame_skip=10, viewer=True):


        self.frame_skip = frame_skip

        model_path = './assets/franka_throw.xml'


        self.model = load_model_from_path(model_path)
        self.sim   = MjSim(self.model, nsubsteps=frame_skip)
        self.data = self.sim.data
        if viewer:
            self.viewer = MjViewer(self.sim)
        self.act_mid = np.mean(self.model.actuator_ctrlrange, axis=1)
        self.act_rng = 0.5 * (self.model.actuator_ctrlrange[:,1] - self.model.actuator_ctrlrange[:,0])

        nu = len(self.act_rng)
        self.action_space = Box(low=-1., high=1., shape=(nu,), dtype=np.float32)

        ob = self.get_obs(self.data)
        self.observation_space = Box(low=-np.inf, high=np.inf, shape=(len(ob),), dtype=np.float32)

        self.obj_bid         = self.model.body_name2id('object')
        self.grasp_sid       = self.model.site_name2id('grasp')
        self.targ_sid        = self.model.site_name2id('target')

        ## -- From the franka examples : 0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4

        self.init_pose = np.array([0., -np.pi/4, 0., -3.*np.pi/4, 0., np.pi/2, np.pi/4])
        init_pose = np.concatenate([self.init_pose, [0]])
        self.def_action = (init_pose - self.act_mid) / self.act_rng * 0

    def reset(self):

        self.sim.reset()
        self.sim.data.qpos[:7] = self.init_pose[:]
        # self.sim.data.qpos[0] = -0.3
        # self.sim.data.qpos[1] = -1.
        # self.sim.data.qpos[3] = -1.7
        # self.sim.data.qpos[5] = 1.
        self.sim.forward()
        ob = self.get_obs(self.data)
        return ob


    def get_obs(self, data):
        return np.concatenate([data.qpos.ravel(), data.qvel.ravel()])


    def get_rew(self, data):
        grasp_pos       = data.site_xpos[self.grasp_sid].ravel()
        grasp_config    = data.site_xmat[self.grasp_sid].reshape(3, 3)

        obj_pos      = data.body_xpos[self.obj_bid].ravel()
        obj_config   = mat2quat(data.body_xmat[self.obj_bid].reshape(3,3))
        target_pos   = data.site_xpos[self.targ_sid].ravel()
        target_config = mat2quat(data.site_xmat[self.targ_sid].reshape(3,3))

        grasp_err       = np.linalg.norm(grasp_pos - obj_pos)

        target_err      = np.linalg.norm(obj_pos - target_pos)

        config_err      = np.linalg.norm(target_config - obj_config)

        return np.log(grasp_err + 0.005)*0. + grasp_err + 10*config_err + 2.*target_err# + np.log(target_err + 0.005)

    # def get_rew(self, data):
    #     grasp_pos       = data.site_xpos[self.grasp_sid].ravel()
    #     grasp_config    = data.site_xmat[self.grasp_sid].reshape(3, 3)
    #
    #     obj_pos      = data.body_xpos[self.obj_bid].ravel()
    #     obj_config   = mat2quat(data.body_xmat[self.obj_bid].reshape(3,3))
    #     target_pos   = data.site_xpos[self.targ_sid].ravel()
    #     target_config = mat2quat(data.site_xmat[self.targ_sid].reshape(3,3))
    #
    #     grasp_err       = np.linalg.norm(grasp_pos - np.array([0.5,0.,0.5]))
    #
    #
    #     return np.log(grasp_err + 0.005) + grasp_err


    def step(self, a):

        ctrl = np.clip(a, -1.0, 1.0)
        ctrl = self.act_mid + ctrl * self.act_rng

        # ctrl[:7] = ctrl[:7]*0 + self.init_pose
        ctrl[-1] = 0
        self.data.ctrl[:] = ctrl

        self.sim.step()

        ob = self.get_obs(self.data)
        rew = self.get_rew(self.data)
        done = False
        return ob, rew, done, {}


    def render(self):
        self.viewer.render()

    def get_state(self):
        return self.sim.get_state()
