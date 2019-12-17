#! /usr/bin/env python

import roslib
roslib.load_manifest('navigation')
import rospy
import actionlib

from navigation.msg import RotateToGoalAction, RotateToGoalGoal, Point_xy


def feedback_cb(feedback):
    print('[Feedback] Time elapsed: %f'%(feedback.distance_left))


if __name__ == '__main__':
    rospy.init_node('rotator_client')
    client = actionlib.SimpleActionClient('rotator', RotateToGoalAction)
    client.wait_for_server()
    print("Done waiting for server")
    goal1 = RotateToGoalGoal()
    goal1.goal = Point_xy([2,2])
    # Fill in the goal here
    client.send_goal(goal1,feedback_cb=feedback_cb)
    # print("Goal sent"+goal1)
    client.wait_for_result(rospy.Duration.from_sec(500.0))
    print("Done waiting for result")
    print('[Result] State: %d'%(client.get_state()))
    print('[Result] Status: %s'%(client.get_goal_status_text()))
    print('[Result] Goal Reached?: %r'%(client.get_result().result))