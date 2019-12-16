#! /usr/bin/env python

import rospy
from RRT import RRT

import utils
from utils import adjustable_random_sampler as sampler
from utils import los_optimizer as path_optimizer

from navigation.srv import Planner , PlannerRequest , PlannerResponse
from navigation.msg import Point_xy, PolyArray, PointArray

class Root():
    """
	Root class for path planning.
	Present Planners->(RRT*, )
    Attributes:
		planning_srv: Sevice-Server for planning 
	"""
    def __init__(self, *args , **kwargs):
		"""
		Initialize all the planning algos. Service to be called --> 'RRT_star'_planner_service  
		"""
		#rrt_star Planner Service 
		self.planning_srv = rospy.Service('rrt_star_planner_service', Planner, self.plan)

    def plan(self , request):
		"""Call Path planner selected(RRT*, )
			
			Args:
			      request: Request by ros client. Contains 
            start_pos: tuple with start point coordinates.
            end_pos: tuple with end point coordinates.
            scan_list: list of obstacles which themselves are list of points
            animation: flag for showing planning visualization (default False)
				

			Returns:
				RETURN_RESP: PlannerResponse()
				
				navigation/PointArray path
					navigation/Point_xy[] points
						float32[] point
				
				bool ack
		"""
		rospy.loginfo("Planning Request received")
		
		#Converting request from ros msg format -> basic python data type

		ST_PT = tuple(request.start_pos.point) 
		END_PT = tuple(request.goal_pos.point)
		SCAN = request.scan_list
		#OBSTACLE = []
		
		#Extracting Obstacle information from ROBST
		#for pt_array in ROBST : 
		#	tmp = []
		#	OBSTACLE.append(tmp)
        
		RETURN_RESP = PlannerResponse()

		print("-"*30)
		rospy.loginfo(" Starting to plan from %r -> %r \n Obstacles-> %r"%(ST_PT,END_PT,SCAN))
		print("-"*30)
		
		#Make class instance and get path
		tree = RRTStar(sample_area=(-5, 15), sampler=sampler, expand_dis=0.1)
		PATH , node_list = tree(ST_PT , END_PT , SCAN)
		
		
		RETURN_RESP.path.points = [Point_xy( [ pts[0] , pts[1] ] ) for pts in PATH]
		RETURN_RESP.ack = True
		print("-"*30)
		return RETURN_RESP

def main():
	rospy.init_node("Planner_Service", anonymous=True)
	try:
		Planner_Service = Root()
		rospy.loginfo("Setup for planners completed!")
	except Exception as er : 
		rospy.logerr("path_planner error: ")
		print(er)

	rospy.spin()
	
if __name__ == "__main__":
	main()
	rospy.logwarn("Killing!")