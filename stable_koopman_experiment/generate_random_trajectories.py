#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from franka_msgs.msg import FrankaState
from tf.transformations import decompose_matrix
import numpy as np
import pickle
from datetime import datetime

class Experiment(object):

    def __init__(self):
        self.hz = 50
        self.rate       = rospy.Rate(self.hz)
        self.jnt_cmd    = JointState()
        self.cmd_pub    = rospy.Publisher("/jnt_cmd", JointState, queue_size=1)
        self.state      = None
        self.eta   = np.zeros(7)
        self.alpha = 0.9
        self.log = {'state' : [], 'action' : [], 'next_state' : []}

        # rospy.Subscriber("/joint_states", JointState, self.callback)
        rospy.Subscriber("/franka_state_controller/franka_states", FrankaState, self.callback)

    def callback(self, msg):
        ee_pose = np.array(msg.O_T_EE).reshape((4,4), order='F')
        scale, shear, euler, pose, _ = decompose_matrix(ee_pose)
        self.state = np.concatenate([pose, euler, msg.q[:7], msg.dq[:7]])

    def run(self):
        max_t = 400
        t = 0
        while not rospy.is_shutdown() and t < max_t:
            if self.state is not None:
                state = self.state.copy()
                self.eta = self.alpha * self.eta + (1-self.alpha) * np.random.normal(0., 0.5, size=(7,))
                self.jnt_cmd.velocity = self.eta.copy()
                self.cmd_pub.publish(self.jnt_cmd)
                self.log['state'].append(state)
                self.log['action'].append(self.eta.copy())
                t += 1
                print('time ', t, 'out of', max_t)
            self.rate.sleep()
        print('Saving data....')
        now = datetime.now()
        date_str = now.strftime("%Y-%m-%d_%H-%M-%S")
        file_name = date_str + '-' + str(self.hz) + '_hz-' + 'data.pkl'
        pickle.dump(self.log, open('./data/' + file_name, 'wb'))
        print('successfully done')
if __name__ == '__main__':
    rospy.init_node('random_cmds')
    experiment = Experiment()
    experiment.run()
