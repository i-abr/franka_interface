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
            [200.0 for _ in range(7)], # lower torque threshold acc
            [200.0 for _ in range(7)], # upper torque threshold acc
            [200.0 for _ in range(7)], # lower torque threshold nominal
            [200.0 for _ in range(7)], # upper torque threshold nominal
            [200.0 for _ in range(6)], # lower force thresholds acc
            [200.0 for _ in range(6)], # upper force thresholds acc
            [200.0 for _ in range(6)], # lower force thresholds acc
            [200.0 for _ in range(6)], # upper force thresholds acc
        )
        if response.success == True:
            print('Shit worked bro')
        else:
            print('This did not work for the following reasons: ')
            print(error)

    except rospy.ServiceException, e:
        print('Service call failed, sucks to suck')



