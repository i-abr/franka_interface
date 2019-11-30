#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Joy

import actionlib

from franka_gripper.msg import GraspAction, GraspGoal, GraspEpsilon

import time

class JoyListener(object):

    def __init__(self):
        rospy.Subscriber('/joy', Joy, self.joy_callback)
        self.delta_x = 0.
        self.delta_y = 0.
        self.delta_z = 0.

        self.grasp   = 1
    def joy_callback(self, msg):
        self.delta_x = msg.axes[1]
        self.delta_y = msg.axes[0]
        self.delta_z = msg.axes[4]

        self.grasp   = int(msg.buttons[5])

if __name__ == '__main__':

    rospy.init_node('joy_interface')

    joy_listener = JoyListener()
    pub = rospy.Publisher('/pose_cmd', Pose, queue_size=1)

    client = actionlib.SimpleActionClient('/franka_gripper/grasp', GraspAction)
    client.wait_for_server()

    grasp_goal = GraspGoal()
    grasp_eps  = GraspEpsilon()
    grasp_eps.inner = 0.005
    grasp_eps.outer = 0.005
    grasp_goal.width = 0.1
    grasp_goal.epsilon = grasp_eps
    grasp_goal.speed   = 0.1
    grasp_goal.force   = 0.001


    hz      = 10.0
    dt      = 1.0/hz
    rate    = rospy.Rate(hz)

    pose_cmd = Pose()

    open_grasp = True

    while not rospy.is_shutdown():

        pose_cmd.position.x = joy_listener.delta_x * 0.3
        pose_cmd.position.y = joy_listener.delta_y * 0.3
        if joy_listener.delta_z > 0.:
            pose_cmd.position.z = 0.
        else:
            pose_cmd.position.z = joy_listener.delta_z * 0.45

        # do the grasp action if not in the grasp
        if joy_listener.grasp == 1 and open_grasp == True:
            grasp_goal.width = 0.06
            client.send_goal(grasp_goal)
            # client.wait_for_result(), dont have to wait for it
            open_grasp = False
        if joy_listener.grasp == 0 and open_grasp == False:
            grasp_goal.width = 0.1
            client.send_goal(grasp_goal)
            # client.wait_for_result()
            open_grasp = True



        pub.publish(pose_cmd)
        rate.sleep()
