#!/usr/bin/env python


import rospy
import rosbag
import torch
import torch.optim as optim
import numpy as np
import argparse
from policy import BasicPolicy

import tf
from geometry_msgs.msg import Pose

import actionlib
from franka_gripper.msg import GraspAction, GraspGoal, GraspEpsilon
import time

import os
import sys

sys.path.append(os.path.dirname(os.path.abspath(__file__)))


if __name__ == '__main__':

    rospy.init_node('behavior_clone_policy')

    expert_demo_file_path = os.path.dirname(os.path.abspath(__file__)) + '/data/expert_demo/policy_param.pt'

    policy = BasicPolicy(6, 4)
    policy.load_state_dict(torch.load(expert_demo_file_path))
    policy.eval()


    tf_listener = tf.TransformListener()
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

        (trans,rot) = tf_listener.lookupTransform('/panda_EE', '/panda_link0', rospy.Time(0))
        (ee_block_trans, rot) = tf_listener.lookupTransform('/EE', '/block', rospy.Time(0))

        action = policy.get_action(np.array(trans + ee_block_trans))

        print(action)

        delta_x = action[0]
        delta_y = action[1]
        delta_z = action[2]
        grasp  = round(action[3])

        pose_cmd.position.y = delta_y * 0.3
        if delta_x < 0:
            pose_cmd.position.x = 0.
        else:
            pose_cmd.position.x = delta_x * 0.24

        if delta_z > 0.:
            pose_cmd.position.z = 0.
        else:
            pose_cmd.position.z = delta_z * 0.45

        # do the grasp action if not in the grasp
        if grasp == 1 and open_grasp == True:
            grasp_goal.width = 0.06
            client.send_goal(grasp_goal)
            # client.wait_for_result(), dont have to wait for it
            open_grasp = False
        if grasp == 0 and open_grasp == False:
            grasp_goal.width = 0.1
            client.send_goal(grasp_goal)
            # client.wait_for_result()
            open_grasp = True


        pub.publish(pose_cmd)
        rate.sleep()
