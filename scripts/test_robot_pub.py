#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import JointState

import lcm
from exlcm import env_state


if __name__ == '__main__':
    pub = rospy.Publisher('/jnt_cmd', JointState, queue_size=1)
    rospy.init_node('test_cmd')
    hz = 10
    dt = 1.0/hz
    rate = rospy.Rate(hz)

    time        = 0.
    time_max    = 1.0
    omega_max   = 1.0

    while not rospy.is_shutdown():
        time += dt
        cmd = JointState()
        exp_ = (time - np.fmod(time, time_max))/time_max
        cycle = np.floor((-1.)**exp_)
        omega = cycle * omega_max / 2.0 * (1.0 - np.cos(2.0 * np.pi / time_max * time))
        cmd.velocity = [0., 0., 0., omega, omega, omega, omega]
        pub.publish(cmd)
        rate.sleep()
