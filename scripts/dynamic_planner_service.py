#!/usr/bin/env python

from navigation.srv import Dynamic_RRT , Dynamic_RRTRequest , Dynamic_RRTResponse
from shapely.geometry import Polygon, Point , LineString
import rospy

def chk_intersection(req):
	line = LineString([req.start_pos,req.goal_pos])
	for obstacle in req.scan_list:
		if line.instersects(Polygon(obstacle)):
			return Dynamic_RRTResponse(False)
	return Dynamic_RRTResponse(True)

def obstacle_check():
	rospy.init_node('obstacle_check')
	s = rospy.Service('dynamic_planner_service', Dynamic_RRT, chk_intersection)
	rospy.spin()


if __name__ == '__main__':
	obstacle_check()
