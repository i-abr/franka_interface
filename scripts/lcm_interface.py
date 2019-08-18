#!/usr/bin/env python

import rospy
import math
import tf
from sensor_msgs.msg import JointState

import lcm
from exlcm import env_state


class StateListener(object):
    def __init__(self):
        rospy.Subscriber('/joint_states', JointState, self.callback)
        self.joint_state = JointState()
    def callback(self, data):
        self.joint_state = data


if __name__ == '__main__':
    rospy.init_node('lcm_tf_listener')
    lc = lcm.LCM()

    listener = tf.TransformListener()
    state_listener = StateListener()

    env_state_msg = env_state()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform('/tag_0', '/tag_1', rospy.Time(0))
            env_state_msg.object_position = trans
            env_state_msg.object_orientation = rot
            lc.publish('env_state_pub', env_state_msg.encode())
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        rate.sleep()
