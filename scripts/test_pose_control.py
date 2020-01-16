#!/usr/bin/env python

import rospy
import numpy as np
import tf
from tf import transformations as trans

from geometry_msgs.msg import Pose

from std_msgs.msg import Float32MultiArray

import lcm
from exlcm import env_state


if __name__ == '__main__':
    rospy.init_node('test_cmd')

    pub = rospy.Publisher('/pose_cmd', Float32MultiArray, queue_size=1)

    cmd = Float32MultiArray()

    hz = 10.0
    dt = 1.0/hz
    rate = rospy.Rate(hz)

    tf_listener = tf.TransformListener()

    init_pose_ = None

    while (init_pose_ is None) and (not rospy.is_shutdown()):
        try:
            (pose_, rot_) = tf_listener.lookupTransform('panda_link0', 'panda_EE', rospy.Time(0))
            if init_pose_ is None:
                init_pose_ = trans.quaternion_matrix(rot_)
                init_pose_[:3,-1] = pose_
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
        rate.sleep()

    print "obtained initiatial pose, testing movement now"

    cmd.position.x = init_pose_[0,-1]
    cmd.position.y = init_pose_[1,-1]
    cmd.position.z = init_pose_[2,-1]

    cmd.data = init_pose_.flatten('F')
    time = 0.
    while not rospy.is_shutdown():
        time += dt
        cmd.data[12] = init_pose_[0,-1] + 0.1 * np.sin(np.pi/2.0 * time)
        pub.publish(cmd)
        rate.sleep()
