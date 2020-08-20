#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from franka_msgs.msg import FrankaState
import numpy as np
import pickle
from datetime import datetime
import scipy.io as sio
from tf.transformations import decompose_matrix


class Basis(object):
    def __init__(self):
        pass
    def __call__(self, x):
        # return np.concatenate([x, np.sin(x), [1.0]])
        return np.concatenate([x, [1]])

class Controller(object):

    def __init__(self):
        self.gain_fname = './notebook/LQR_gains_stableA_and_B.mat'
        self.Klqr = sio.loadmat(self.gain_fname, squeeze_me=True)['K_gain']

    def __call__(self, x, x_t):
        return -np.dot(self.Klqr, x - x_t)

class Experiment(object):

    def __init__(self):
        self.hz = 50
        self.rate       = rospy.Rate(self.hz)
        self.jnt_cmd    = JointState()
        self.cmd_pub    = rospy.Publisher("/jnt_cmd", JointState, queue_size=1)
        self.state      = None
        self.eta   = np.zeros(7)
        self.alpha = 0.9
        self.log = {'state' : [], 'action' : [], 'target_state' : []}
        self.basis = Basis()
        #rospy.Subscriber("/joint_states", JointState, self.callback)
        rospy.Subscriber("/franka_state_controller/franka_states", FrankaState, self.callback)
        
        self.target_states = np.load('./notebook/desired_trajectory.npy')[:,:-1]

        self.target_z_state = self.basis(self.target_states[0])
        self.controller = Controller()

    def callback(self, msg):
        ee_pose = np.array(msg.O_T_EE).reshape((4,4), order='F')
        scale, shear, euler, pose, _ = decompose_matrix(ee_pose)
        self.state = np.concatenate([pose, euler, msg.q[:7], msg.dq[:7]])

    def run(self):
        t = 0
        max_t = self.target_states.shape[0]
        while not rospy.is_shutdown() and t < max_t:
            if self.state is not None:
                self.target_z_state = self.basis(self.target_states[t])
                state = self.state.copy()
                u = self.controller(self.basis(state), self.target_z_state)
                self.jnt_cmd.velocity = u
                self.cmd_pub.publish(self.jnt_cmd)
                self.log['state'].append(state)
                self.log['action'].append(u)
                t += 1
                print('time ', t, 'out of', max_t)
            self.rate.sleep()

        print('Saving data....')
        now = datetime.now()
        date_str = now.strftime("%Y-%m-%d_%H-%M-%S")
        file_name = self.controller.gain_fname.split('.')[0] + date_str + '-' + str(self.hz) + '_hz-' + 'data.pkl'
        pickle.dump(self.log, open('./joint_tracking_experiment_data/' + file_name, 'wb'))
        print('successfully done')
if __name__ == '__main__':
    rospy.init_node('random_cmds')
    experiment = Experiment()
    experiment.run()
