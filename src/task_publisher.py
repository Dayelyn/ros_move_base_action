#!/usr/bin/env python

import rospy
from std_msgs.msg import String


def task_publisher(task_input):
    pub = rospy.Publisher('ltl_task', String, queue_size=10)
    rospy.init_node('task_publisher', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pub.publish(task_input)
        rate.sleep()


if __name__ == '__main__':
    try:
        #hard_task = '(<>(r5 && <>(r4 && <>r1)) && []!r2)'
        hard_task = '(<>(r3 && <>r1))'
        soft_task = ''
        total_task = hard_task + ',' + soft_task
        task_publisher(total_task)
        # Simply publish hard task and soft task. It will be linked to the GUI in the future
    except rospy.ROSInterruptException:
        rospy.loginfo('No task is being published!')
