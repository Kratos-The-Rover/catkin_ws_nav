#!/usr/bin/env python

import sys
import rospy
from navigation.srv import *
from navigation.msg import *
import numpy as np
import os
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

# from context import RRT, utils
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), './rrt_for_scan')))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), './rrt_star')))
from utils_scan import scan_obstacle_checker, make_obstacles_scan, check_intersection_scan
from utils_scan import adjustable_random_sampler as sampler
from descartes import PolygonPatch
from shapely.geometry import Polygon, Point, LineString
import random
import math, time
import matplotlib.pyplot as plt


sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/./rrt_for_scan/")
sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/./rrt_star/")



try:
    from rrt import RRT
    #from rrt_star_scan1 import RRTSTar
except ImportError:
    raise

show_animation = True

start_time = 0
show_animation=True
def dynamic_client(x, y,z):
    print("Starting dynamic_client")
    rospy.wait_for_service('dynamic_planner_service')
    try:
        check_intersect = rospy.ServiceProxy('dynamic_planner_service', Dynamic)
        print("Proxy created")
        resp1 = check_intersect(x, y, z)
        # print(resp1)
        return resp1.ack
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e


if __name__ == "__main__":
    inf=100
    nan=1000
    scan_list =[1.309682011604309, 1.3101788759231567, 1.310431718826294, 1.3106871843338013, 1.3109458684921265, 1.311471939086914, 1.311739444732666, 1.3120099306106567, 1.3122836351394653, 1.3125602006912231, 1.3128397464752197, 1.3134082555770874, 1.313697099685669, 1.3139890432357788, 1.3142839670181274, 1.3145822286605835, 1.3151880502700806, 1.3154956102371216, 1.3158066272735596, 1.3161206245422363, 1.316437840461731, 1.316758394241333, 1.3174091577529907, 1.317739486694336, 1.3180732727050781, 1.318410038948059, 1.3187503814697266, 1.3190940618515015, 1.3194409608840942, 1.3201451301574707, 1.3205022811889648, 1.3208627700805664, 1.3212268352508545, 1.3215943574905396, 1.3219653367996216, 1.3223397731781006, 1.3230992555618286, 1.3234843015670776, 1.3238729238510132, 1.3242650032043457, 1.3246607780456543, 1.3250601291656494, 1.325463056564331, 1.3258696794509888, 1.3266940116882324, 1.3271117210388184, 1.32753324508667, 1.327958583831787, 1.3283874988555908, 1.3288201093673706, 1.3292566537857056, 1.3296970129013062, 1.3301411867141724, 1.3310409784317017, 1.3314965963363647, 1.331956148147583, 1.332419753074646, 1.3328871726989746, 1.3333585262298584, 1.333833932876587, 1.3343132734298706, 1.3347965478897095, 1.335283875465393, 1.3362706899642944, 1.3367700576782227, 1.3372735977172852, 1.3377811908721924, 1.3382928371429443, 1.3388087749481201, 1.3393288850784302, 1.339853048324585, 1.340381383895874, 1.3409141302108765, 1.3414509296417236, 1.3419921398162842, 1.342537522315979, 1.3436414003372192, 1.3441998958587646, 1.3447626829147339, 1.3453298807144165, 1.345901370048523, 1.3464775085449219, 1.347057819366455, 1.3476428985595703, 1.3482322692871094, 1.3488261699676514, 1.3494247198104858, 1.3500275611877441, 1.3506351709365845, 1.3512473106384277, 1.3518640995025635, 1.3524854183197021, 1.3531115055084229, 1.353742241859436, 1.3543776273727417, 1.35566246509552, 1.3563120365142822, 1.356966495513916, 1.3576256036758423, 1.3582897186279297, 1.3589587211608887, 1.3596327304840088, 1.360311508178711, 1.3609951734542847, 1.361683964729309, 1.362377405166626, 1.363076090812683, 1.3637796640396118, 1.3644883632659912, 1.3652020692825317, 1.365920901298523, 1.3666447401046753, 1.3673737049102783, 1.3681076765060425, 1.3688467741012573, 1.3695908784866333, 1.370340347290039, 1.3710949420928955, 1.3718549013137817, 1.3726199865341187, 1.3733903169631958, 1.3741658926010132, 1.3749470710754395, 1.375733494758606, 1.3765252828598022, 1.3773225545883179, 1.3781251907348633, 1.378933310508728, 1.379746913909912, 1.380566120147705, 1.3813908100128174, 1.382220983505249, 1.3830569982528687, 1.3838989734649658, 1.3847463130950928, 1.3855993747711182, 1.386458158493042, 1.3873226642608643, 1.3881930112838745, 1.3890687227249146, 1.3899502754211426, 1.3908374309539795, 1.3917304277420044, 1.3926293849945068, 1.3935341835021973, 1.3944448232650757, 1.3953614234924316, 1.3962838649749756, 1.3972123861312866, 1.3981468677520752, 1.3990874290466309, 1.4000341892242432, 1.4009875059127808, 1.401947021484375, 1.4029126167297363, 1.4038845300674438, 1.404862403869629, 1.4058465957641602, 1.406836986541748, 1.4078336954116821, 1.4088366031646729, 1.4098458290100098, 1.4108614921569824, 1.4118835926055908, 1.4129116535186768, 1.4139459133148193, 1.4149869680404663, nan, 1.41603422164917, 1.4170881509780884, 1.4181488752365112, 1.4192163944244385, 1.4202903509140015, 1.4213711023330688, 1.4224584102630615, 1.4235525131225586, 1.42465341091156, 1.4257612228393555, 1.4268758296966553, 1.4279974699020386, 1.4291259050369263, 1.4302610158920288, 1.4314026832580566, 1.4325510263442993, 1.433706283569336, 1.434868574142456, 1.4360378980636597, 1.4372141361236572, 1.438397765159607, 1.439588189125061, nan, 1.4407858848571777, 1.441990613937378, 1.4432027339935303, 1.4444221258163452, 1.4456491470336914, 1.4468833208084106, 1.4481250047683716, 1.4493738412857056, 1.4506300687789917, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan]
   
    start_pose = Point_xy([0,0])
    goal_pose = Point_xy([4,4])
   
    #print "Requesting %s+%s"%(x, y)
    intersection = dynamic_client(start_pose,goal_pose,scan_list)
    
    if intersection:
        print("There is no intersection.")
    else:
        print("There is an intersection.")
       # final_path = path

        