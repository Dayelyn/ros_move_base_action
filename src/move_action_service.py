#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_action.srv import * # NextMove, NextMoveResponse, MoveAct, MoveActResponse
# srv generate Response, need to be included


def act_move_base_client(pose_input, index):

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose = pose_input[index]

    client.send_goal(goal)
    client.wait_for_result()

    return client.get_result()


def srv_next_move_planner_client(PoseMode):

    try:
        rospy.wait_for_service('next_move_planner_service')
        pose_loader = rospy.ServiceProxy('next_move_planner_service', NextMove)
        pose_list = pose_loader(PoseMode)
        return pose_list.poses

    except rospy.ServiceException:
        print "Service call failed"


def move_base_action(req):
    try:
        pose_list = srv_next_move_planner_client(req.status)
        index = 0

        while not rospy.is_shutdown():

            result = act_move_base_client(pose_list, index)

            if result and (index <= len(pose_list) - 2):
                rospy.loginfo("Goal[%s] execution done!", index)
                index = index + 1
            else:
                info = "Navi11111finished!"
                rospy.loginfo(info)
                return MoveActResponse('Navi finished!')

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation interrupted.")


def move_base_action_server():
    rospy.init_node('move_base_action_server')
    rospy.Service('move_base_action_service', moveactsrv, move_base_action)
    rospy.spin()


if __name__ == '__main__':
    move_base_action_server()

