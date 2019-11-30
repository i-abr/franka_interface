#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Pose

import lcm
from exlcm import env_state


if __name__ == '__main__':
    pub = rospy.Publisher('/pose_cmd', Pose, queue_size=1)
    rospy.init_node('test_cmd')
    hz = 1000.0
    dt = 1.0/hz
    rate = rospy.Rate(hz)

    time        = 0.
    time_max    = 1.0
    omega_max   = 1.0

    radius = 0.3


    while not rospy.is_shutdown():
        time += dt
        cmd = Pose()
        angle = np.pi/2.0 * (1 - np.cos(np.pi/5.0 * time))
        delta_x = radius * np.sin(angle)
        delta_z = radius * (np.cos(angle) - 1)
        # // double radius = 0.3;
        # // double angle = M_PI / 4 * (1 - std::cos(M_PI / 5.0 * elapsed_time_.toSec()));
        # // double delta_x = radius * std::sin(angle);
        # // double delta_z = radius * (std::cos(angle) - 1);
        # // std::array<double, 16> new_pose = initial_pose_;
        cmd.position.x = delta_x
        cmd.position.z = delta_z

        pub.publish(cmd)
        rate.sleep()
