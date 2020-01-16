#!/usr/bin/env python

import rospy
import numpy as np
import tf
from tf import transformations as trans
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy
from franka_msgs.msg import FrankaState

def state_callback(msg):
    print msg.O_F_ext_hat_K

class JoyListener(object):

    def __init__(self):
        rospy.Subscriber('/joy', Joy, self.joy_callback)
        self.delta_x = 0.
        self.delta_y = 0.
        self.delta_z = 0.
        self.delta_rot_x = 0.
        self.delta_rot_y = 0.
        self.delta_rot_z = 0.
        self.grasp   = False

    def joy_callback(self, msg):
        self.delta_x = msg.axes[1]
        self.delta_y = msg.axes[0]
        self.delta_z = msg.axes[4]
        self.delta_rot_x = -msg.axes[3]
        self.delta_rot_y = -msg.axes[4]
        self.delta_rot_z = -msg.axes[5]
        self.grasp   = int(msg.buttons[5])


def_pose = np.array([
    [1., 0., 0., .3],
    [0.,-1., 0., 0.],
    [0., 0.,-1., .4],
    [0., 0., 0., 1.]
])

if __name__ == '__main__':
    rospy.init_node('test_cmd')
    rospy.Subscriber('/franka_state_controller/franka_states', FrankaState, state_callback)
    pub = rospy.Publisher('/pose_cmd', Float32MultiArray, queue_size=1)
    joy_listener = JoyListener()

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

    # cmd.data = init_pose_.flatten('F')


    # cmd.data = np.dot(def_pose, new_rot).flatten('F')
    cmd.data = def_pose.flatten('F')

    time = 0.
    while not rospy.is_shutdown():
        time += dt

        rotx = trans.rotation_matrix(25.*np.pi/180. * joy_listener.delta_rot_x, np.array([1., 0., 0.]))
        roty = trans.rotation_matrix(25.*np.pi/180. * joy_listener.delta_rot_y, np.array([0., 1., 0.]))
        cmd.data = np.dot(def_pose, np.dot(rotx, roty)).flatten('F')
        pub.publish(cmd)
        rate.sleep()
