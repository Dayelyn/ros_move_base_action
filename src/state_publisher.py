#!/usr/bin/env python

import rospy
from move_action.srv import *


def srv_move_base_action_client(PoseMode):
    try:
        rospy.wait_for_service('move_base_action_service')
        parapass = rospy.ServiceProxy('move_base_action_service', MoveAct)
        info = parapass(PoseMode)
        return info.result

    except rospy.ServiceException:
        print "Service call failed"


if __name__ == '__main__':
    rospy.init_node('action_client')
    move = srv_move_base_action_client('PoseQueue')
    rospy.loginfo(move)
    rospy.spin()









