#!/usr/bin/env python

import rospy 
from franka_control.srv import SetFullCollisionBehavior

if __name__ == '__main__':
    
    rospy.wait_for_service('/franka_control/set_full_collision_behavior')

    try:
        update_collision_behavior = rospy.ServiceProxy(
                            '/franka_control/set_full_collision_behavior',
                            SetFullCollisionBehavior)

        response = update_collision_behavior(
            [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0], # lower torque threshold acc
            [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0], # upper torque threshold acc
            [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0], # lower torque threshold nominal
            [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0], # upper torque threshold nominal
            [20.0, 20.0, 20.0, 25.0, 25.0, 25.0], # lower force thresholds acc
            [20.0, 20.0, 20.0, 25.0, 25.0, 25.0], # upper force thresholds acc
            [20.0, 20.0, 20.0, 25.0, 25.0, 25.0], # lower force thresholds acc
            [20.0, 20.0, 20.0, 25.0, 25.0, 25.0], # upper force thresholds acc
        )
        if response.success == True:
            print('Shit worked bro')
        else:
            print('This did not work for the following reasons: ')
            print(error)

    except rospy.ServiceException, e:
        print('Service call failed, sucks to suck')



