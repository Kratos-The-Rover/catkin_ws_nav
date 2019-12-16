#!/usr/bin/env python
from navigation.srv import Dynamic_RRT,Dynamic_RRTResponse
import rospy
import numpy as np
import sys
import os
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
# from context import RRT, utils
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../rrt_for_scan')))

from utils_scan import scan_obstacle_checker, make_obstacles_scan, check_intersection_scan
from utils_scan import adjustable_random_sampler as sampler
from descartes import PolygonPatch
from shapely.geometry import Polygon, Point, LineString
import random
import math, time
import matplotlib.pyplot as plt

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../rrt_for_scan/")



try:
    from rrt import RRT
except ImportError:
    raise

show_animation = True




def handle_add_two_ints(req):
    #print "Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b))
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    print("Start " + __file__)
    scan_list = req.input
    obstacle_list = req.path
    line_obstacles, pts = make_obstacles_scan(scan_list)
    line_obstacles=line_obstacles+obstacle_list
    
   
    return Dynamic_RRTResponse(line_obstacles)

def add_two_ints_server():
    rospy.init_node('dynamic_rrt_server')
    s = rospy.Service('dynamic_rrt', Dynamic_RRT, handle_add_two_ints)
    print "Ready."
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()


