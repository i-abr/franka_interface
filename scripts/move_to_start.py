#!/usr/bin/env python

import rospy
from moveit_commander import MoveGroupCommander
from actionlib_msgs.msg import GoalStatusArray

if __name__ == '__main__':
    rospy.init_node('move_to_start')
    rospy.wait_for_message('move_group/status', GoalStatusArray)
    starting_joints = rospy.get_param('starting_joints')
    commander = MoveGroupCommander('panda_arm')
    commander.set_joint_value_target(starting_joints)
    #commander.set_named_target('ready')
    commander.go()
