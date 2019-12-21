#! /usr/bin/env python
"""
Path planning Code of RRT* with
author: Ojit Mehta(@ojitmehta123)
"""
import numpy as np
import sys
import os

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


def propagate_cost_to_leaves(node_list, parent_node):

        for node in node_list:
            if node.parent == parent_node:
                node.cost = calculate_cost(parent_node, (node.x, node.y))
                propagate_cost_to_leaves(node_list, node)


def calculate_cost(from_node, to_node):
    """ To calculate Cost from from_node --> to_node
        Arguments: 	from_node --> Node from node_list
                    to_node --> tuple
        Return:
                    Value of Cost from start to to_node
    """
    return from_node.cost + math.sqrt((from_node.x - to_node[0])**2 + (from_node.y - to_node[1])**2)


def find_near_nodes(node_list, new_node, circle_dist):
    """ To Find the nearest nodes at max circle_dist from new_node in node_list from new_node  """
    nnode = len(node_list) + 1
    r = circle_dist * math.sqrt((math.log(nnode) / nnode))
    dist_list = [(node.x - new_node[0]) ** 2 +
                    (node.y - new_node[1]) ** 2 for node in node_list]
    near_inds = [dist_list.index(i) for i in dist_list if i <= r ** 2]
    return near_inds


class Node(object):
    """
    Coordinate representation in node form.
    x,y --> Coordinates
    Parent node is the node connected to the present node
    """

    def __init__(self, x, y):
            self.x = x
            self.y = y
            self.path_x = []
            self.path_y = []
            self.parent = None
            self.cost = 0.0

    def __str__(self):
        return ("("+str(self.x)+','+str(self.y)+")")




class RRTStar(object):
    """
    RRT star algorithm
    """

    def __init__(self, sample_area,
                    expand_dis=0.5,
                    path_resolution=1.0,
                    goal_sample_rate=0.1,
                    max_iter=250,
                    connect_circle_dist=20.0
                    ):
        """
        start: Start Point. in our case remains(0 , 0) unless specified
        goal: Next goal to be reached
        scan = LaserScan polar distances to Obstacles [r1,r2,r3...] initially assuming every scan occurs at 1 rad interval
        randomArea:Random Sampling Area
        """

        self.sample_area = sample_area
        self.sampler = sampler
        self.expand_dis = expand_dis
        self.goal_sample_rate = goal_sample_rate
        self.circle = connect_circle_dist
        self.max_iter = max_iter

    def __call__(self, goal_point, scan, start_point=[0, 0], animation=True):
        """Plans path from start to goal avoiding obstacles.

        Args:
            start_point: tuple with start point coordinates.
            end_point: tuple with end point coordinates.
            scan: list of obstacles which themselves are list of points
            animation: flag for showing planning visualization (default False)

        Returns:
            A list of points representing the path determined from
            start to goal while avoiding obstacles.
            An list containing just the start point means path could not be planned.
        """
        search_until_max_iter = True

        # Make line obstacles and scan in x,y from scan
        line_obstacles, pts = make_obstacles_scan(scan)

        # Setting Start and End
        self.start = Node(start_point[0], start_point[1])
        self.goal = Node(goal_point[0], goal_point[1])

        # Initialize node with Starting Position
        self.node_list = [self.start]

        # Loop for maximum iterations to get the best possible path
        for iter in range(self.max_iter):
            ########################################
            # print("ITER-->")
            # print(iter)
            ########################################

            #########################################
            # print("NODE_LIST-->")
            # for printer_i in self.node_list:
            #     print(printer_i)
            #########################################
            # Sample a Random point in the sample area
            rnd_point = sampler(self.sample_area, (self.goal.x , self.goal.y), self.goal_sample_rate)

            ########################################
            # print("RANDOM POINT-->")
            # print(rnd_point)
            ########################################

            # Find nearest node to the sampled point
            distance_list = [(node.x - rnd_point[0])**2 + (node.y -
                              rnd_point[1])**2 for node in self.node_list]
            nearest_node = self.node_list[distance_list.index(min(distance_list))]
            ########################################
            # print("NEAREST_NODE-->")
            # print(nearest_node.x , nearest_node.y , nearest_node.cost)
            ########################################
            # Creating a new Point in the Direction of sampled point
            theta = math.atan2(rnd_point[1] - nearest_node.y,
                               rnd_point[0] - nearest_node.x)
            new_point = nearest_node.x + self.expand_dis*math.cos(theta), \
                        nearest_node.y + self.expand_dis*math.sin(theta)
            
            #########################################
            # print("NEW_POINT-->")
            # print(new_point[0],new_point[1])
            #########################################
            # Check obstacle collision
            new_point = scan_obstacle_checker(scan, new_point)

            if math.isnan(new_point[0]):
                ########################################
                #print("ISNAN-->"+str(iter))
                # print(new_point)
                ########################################
                continue

            #If iterations is less than certain no. try exploring a bit    
            if iter<20:
                new_node = Node(new_point[0],new_point[1])
                new_node.parent = nearest_node
                new_node.cost = nearest_node.cost + math.sqrt((new_node.x-nearest_node.x)**2 + (new_node.y-nearest_node.y)**2)
                
                #Set the path for new node
                present_node = new_node
                px =[]
                py=[]
                while present_node.parent != None:
                    px.append(present_node.x)
                    py.append(present_node.y)
                    present_node = present_node.parent
                px.append(self.start.x)
                py.append(self.start.y)
                new_node.path_x = px[:]
                new_node.path_y = py[:]
                self.node_list.append(new_node)
                if animation and iter % 5 == 0:
                    self.draw_graph(scan, new_node)

                continue

            nearest_indexes = find_near_nodes(self.node_list, new_point, self.circle)

            # Getting the parent node from nearest indices

            costs = []  # List of Total costs from the start to new_node when attached to parent node in node_list
            temp_points = []

            for index in nearest_indexes:
                near_node = self.node_list[index]
                point_list = [(near_node.x , near_node.y), (new_point[0],new_point[1])]
                if not check_intersection_scan(point_list, line_obstacles):
                    costs.append(near_node.cost + math.sqrt((near_node.x - new_point[0])**2 + (near_node.y - new_point[1])**2))
                else:
                    costs.append(float("inf"))
            
            min_cost = min(costs)
            # Calculating the minimum cost and selecting the node for which it occurs as parent child

            if min_cost == float("inf"):
                continue

            # Setting the new node as the one with min cost
            min_ind = nearest_indexes[costs.index(min_cost)]
            new_node = Node(new_point[0],new_point[1])
            new_node.parent = self.node_list[min_ind]
            new_node.cost = min_cost

            #########################################
            # print("NEW_NODE-->")
            # print(new_node.x , new_node.y , new_node.cost)
            #########################################

            if new_node:
                    
                self.node_list.append(new_node)
                
                for ind in nearest_indexes:
                    node_check = self.node_list[ind]
                    point_list = [(new_node.x , new_node.y), (node_check.x , node_check.y)]
                    
                    no_coll = not check_intersection_scan(point_list, line_obstacles)
                    cost_improv = new_node.cost + math.sqrt((new_node.x - node_check.x)**2 + (new_node.y - node_check.y)**2) < node_check.cost

                    if no_coll and cost_improv:
                        node_check.parent = new_node

                present_node = new_node
                px =[]
                py=[]
                while present_node.parent != None:
                    px.append(present_node.x)
                    py.append(present_node.y)
                    present_node = present_node.parent
                px.append(self.start.x)
                py.append(self.start.y)
                new_node.path_x = px[:]
                new_node.path_y = py[:]
            if animation and iter % 5 == 0:
                self.draw_graph(scan, new_node)

            if (not search_until_max_iter) and new_node:  # check reaching the goal
                last_index = self.search_best_goal_node(scan)
                if last_index:
                    path = [[self.goal.x, self.goal.y]]
                    node = self.node_list[last_index]
                    while node.parent is not None:
                        path.append([node.x, node.y])
                        node = node.parent
                    path.append([node.x, node.y])
                    return path


        last_index = self.search_best_goal_node(scan)
        if last_index:
            path = [[self.goal.x, self.goal.y]]
            node = self.node_list[last_index]
            while node.parent is not None:
                path.append([node.x, node.y])
                node = node.parent
            path.append([node.x, node.y])
            return path
        return None

    def draw_graph(self, scan, rnd=None):
        plt.clf()
        pt_ang = np.arange(-0.521567881107,0.524276316166,0.00163668883033)
        pt_scan = np.array(scan)
        pts = []
        pt_x = np.multiply(pt_scan,np.cos(pt_ang))
        pt_y = np.multiply(pt_scan,np.sin(pt_ang))

        for a,b in zip(pt_x,pt_y):
		    pts.append((a,b))

        if rnd is not None:
            plt.plot(rnd.x, rnd.y, "^k")
        for node in self.node_list:
            if node.parent:
                plt.plot(node.path_x, node.path_y, "-g")

        # for (ox, oy, size) in self.obstacle_list:
        #     self.plot_circle(ox, oy, size)
        plt.plot([x for (x, _) in pts], [y for (_, y) in pts],'r.')
        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.goal.x, self.goal.y, "xr")
        plt.axis("equal")
        plt.axis((-5,5,-5,5))
        plt.grid(True)
        plt.pause(0.01)

    def search_best_goal_node(self,scan):
        dist_to_goal_list = [math.sqrt(
            (n.x - self.goal.x)**2 + (n.y - self.goal.y)**2) for n in self.node_list]
        goal_inds = [dist_to_goal_list.index(
            i) for i in dist_to_goal_list if i <= self.expand_dis]

        safe_goal_inds = []
        for goal_ind in goal_inds:
            theta = math.atan2(
                self.node_list[goal_ind].y - self.goal.y, self.node_list[goal_ind].x - self.goal.x)

            t_node = Node(self.node_list[goal_ind].x + math.cos(theta) , self.node_list[goal_ind].y + math.sin(theta))
            if scan_obstacle_checker(scan , (t_node.x , t_node.y)):
                safe_goal_inds.append(goal_ind)

        if not safe_goal_inds:
            return None

        min_cost = min([self.node_list[i].cost for i in safe_goal_inds])
        for i in safe_goal_inds:
            if self.node_list[i].cost == min_cost:
                return i

        return None


def main():
    print("Start " + __file__)
    inf  = 100
    nan = 1000
    
    # ====Search Path with RRT====
    scan_list=[nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, nan, 2.107834577560425, 2.107160806655884, 2.1064937114715576, 2.1058337688446045, 2.1051809787750244, 2.1045351028442383, 2.103896141052246, 2.103264093399048, nan, 2.1026389598846436, 2.1020209789276123, 2.101409673690796, 2.1008055210113525, 2.100208282470703, 2.0996181964874268, 2.0990347862243652, 2.098458766937256, 2.0978894233703613, 2.0973269939422607, 2.096771717071533, nan, 2.0962231159210205, 2.095681667327881, 2.095147132873535, 2.0946195125579834, 2.0940988063812256, 2.093585252761841, 2.093078374862671, 2.092578887939453, 2.092085838317871, 2.091600179672241, nan, 2.091121196746826, 2.090649127960205, 2.090184211730957, 2.089725971221924, 2.0892746448516846, 2.0888304710388184, 2.088393211364746, 2.0879628658294678, 2.0875394344329834, 2.087122917175293, nan, 2.0867130756378174, 2.086310386657715, 2.0859148502349854, 2.0855259895324707, 2.085143804550171, 2.084768772125244, 2.0844006538391113, 2.0840394496917725, 2.0836851596832275, nan, 2.0833375453948975, 2.0829970836639404, 2.0826635360717773, 2.082336664199829, 2.082016706466675, 2.0817036628723145, 2.081397533416748, 2.0810983180999756, 2.080805778503418, 2.0805203914642334, nan, 2.0802416801452637, 2.079969644546509, 2.079704761505127, 2.07944655418396, 2.079195261001587, 2.078950881958008, 2.0787131786346436, 2.0784823894500732, 2.078258514404297, 2.0780413150787354, nan, 2.077831268310547, 2.077627658843994, 2.0774309635162354, 2.0772411823272705, 2.0770578384399414, 2.0768816471099854, 2.076712131500244, 2.076549530029297, 2.0763936042785645, 2.076244354248047, nan, 2.0761020183563232, 2.0759663581848145, 2.0758376121520996, 2.0757157802581787, 2.0756001472473145, 2.075491428375244, 2.075389862060547, 2.0752947330474854, 2.0752062797546387, 2.075124502182007, nan, 2.075049877166748, 2.074981689453125, 2.0749199390411377, 2.0748651027679443, 2.074817180633545, 2.074775457382202, 2.0747408866882324, 2.0747127532958984, 2.0746912956237793, 2.074676752090454, nan, 2.0746684074401855, 2.07466721534729, 2.074672222137451, 2.0746841430664062, 2.074702739715576, 2.074728012084961, 2.0747594833374023, 2.0747978687286377, 2.074842929840088, 2.074894428253174, 2.0749526023864746, nan, 2.075017213821411, 2.0750885009765625, 2.075166702270508, 2.0752511024475098, 2.0753421783447266, 2.075439691543579, 2.0755438804626465, 2.0756545066833496, 2.0757718086242676, 2.075895309448242, 2.0760254859924316, 2.076162338256836, nan, 2.076305627822876, 2.0764551162719727, 2.0766115188598633, 2.0767741203308105, 2.0769431591033936, 2.0771188735961914, 2.077301025390625, 2.0774893760681152, 2.0776846408843994, 2.077885866165161, 2.0780935287475586, 2.078307867050171, nan, 2.078528642654419, 2.0787556171417236, 2.078989028930664, 2.0792288780212402, 2.079475164413452, 2.0797276496887207, 2.079986572265625, 2.080251932144165, 2.0805232524871826, 2.080801248550415, 2.081085443496704, 2.081376075744629, 2.0816729068756104, 2.0819759368896484, nan, 2.0822854042053223, 2.082601308822632, 2.082923412322998, 2.083251476287842, 2.0835862159729004, 2.0839269161224365, 2.0842738151550293, 2.0846269130706787, 2.084986448287964, 2.0853519439697266, 2.085724115371704, 2.086101770401001, 2.0864861011505127, 2.086876630783081, 2.087273120880127, nan, 2.0876755714416504, 2.0880844593048096, 2.0884995460510254, 2.0889205932617188, 2.0893478393554688, 2.0897810459136963, 2.0902204513549805, 2.0906660556793213, 2.0911173820495605, 2.0915751457214355, 2.092038631439209, 2.092508316040039, 2.092984199523926, 2.09346604347229, 2.0939536094665527, 2.094447612762451, 2.094947338104248, 2.0954527854919434, 2.0959646701812744, nan, 2.096482515335083, 2.097005844116211, 2.0975356101989746, 2.0980710983276367, 2.0986125469207764, 2.0991599559783936, 2.099713087081909, 2.1002721786499023, 2.100837230682373, 2.1014082431793213, 2.101984977722168, 2.102567672729492, 2.1031558513641357, 2.103750228881836, 2.1043505668640137, 2.1049561500549316, 2.1055679321289062, 2.1061854362487793, 2.106808662414551, 2.1074376106262207, 2.108072519302368, 2.108712911605835, 2.1093592643737793, 2.110011100769043, 2.110668659210205, 2.1113321781158447, 2.112001419067383, 2.112675905227661, 2.113356351852417, 2.1140425205230713, 2.114734172821045, 2.115431308746338, 2.1161344051361084, 2.116842746734619, 2.1175570487976074, 2.118276834487915, 2.119002103805542, 2.1197328567504883, nan, 2.120469570159912, 2.121211290359497, 2.1219587326049805, 2.1227118968963623, 2.1234705448150635, 2.124234437942505, 2.1250040531158447, 2.125778913497925, 2.1265594959259033, 2.127345323562622, 2.12813663482666, 2.1289334297180176, 2.1297357082366943, 2.1305432319641113, 2.1313562393188477, 2.132174491882324, 2.13299822807312, 2.1346616744995117, 2.1355016231536865, 2.1363468170166016, 2.137197256088257, 2.1380527019500732, 2.138913869857788, 2.139780044555664, 2.1406514644622803, 2.1415281295776367, 2.1424102783203125, 2.1432976722717285, 2.1441900730133057, 2.145087480545044, 2.1459903717041016, 2.1468987464904785, 2.1478118896484375, 2.1487302780151367, 2.149653911590576, 2.1505825519561768, 2.1515161991119385, 2.1524550914764404, 2.1533989906311035, 2.154348134994507, 2.1553022861480713, 2.156261444091797, 2.1572256088256836, 2.1581950187683105, 2.1591691970825195, 2.1601486206054688, 2.161133050918579, 2.1621222496032715, 2.163116455078125, 2.1641159057617188, 2.1651201248168945, 2.1671435832977295, 2.1681623458862305, 2.1691863536834717, 2.170215129852295, 2.1712486743927, 2.1722872257232666, 2.173330783843994, 2.1743788719177246, 2.1754322052001953, 2.176490068435669, 2.1775527000427246, 2.1786203384399414, 2.179692506790161, 2.180769920349121, 2.181851625442505, 2.1829380989074707, 2.1851253509521484, 2.1862261295318604, 2.1873316764831543, 2.1884419918060303, 2.18955659866333, 2.190676212310791, 2.191800355911255, 2.192929267883301, 2.1940627098083496, 2.1952004432678223, 2.196343183517456, 2.1974904537200928, 2.199798583984375, 2.2009594440460205, 2.202124834060669, 2.2032949924468994, 2.2044694423675537, 2.20564866065979, 2.206831932067871, 2.208019733428955, 2.209212303161621, 2.21040940284729, 2.2128162384033203, 2.214026689529419, 2.2152411937713623, 2.2164602279663086, 2.2176835536956787, 2.2189114093780518, 2.2201435565948486, 2.2213799953460693, 2.2238659858703613, 2.2251155376434326, 2.2263691425323486, 2.2276272773742676, 2.2288894653320312, 2.230156183242798, 2.231426954269409, 2.2327020168304443, 2.235264778137207, 2.2365527153015137, 2.237844467163086, 2.239140510559082, 2.240440845489502, 2.2417452335357666, 2.243053913116455, 2.245683193206787, 2.247004270553589, 2.2483294010162354, 2.2496583461761475, 2.2509915828704834, 2.252328872680664, 2.2550153732299805, 2.2563648223876953, 2.257718086242676, 2.259075403213501, 2.260436773300171, 2.2618021965026855, 2.264544725418091, 2.2659218311309814, 2.267302989959717, 2.268688201904297, 2.2700769901275635, 2.272866725921631, 2.2742671966552734, 2.2756717205047607, 2.2770798206329346, 2.278491973876953, 2.2799081802368164, 2.2827513217926025, 2.2841787338256836, 2.2856099605560303, 2.2870450019836426, 2.2884836196899414]
    # Set Initial parameters
    rrt_star = RRTStar(sample_area=[-10, 10])

    print('\n ' + '-'*30 +  "\n> Starting operation ...\n " + '-'*30 + '\n')
    start_time = time.time()

    path = rrt_star(goal_point = [5.917878300568243, -1.7143409531979132], scan = scan_list)

    print('\n ' + '-'*30 + "\n> Time taken: {:.4} seconds.\n ".format(time.time() - start_time) + '-'*30 + '\n')

    if path is None:
        print("Cannot find path")
    else:
        print("found path!!")

        # Draw final path
        if show_animation:
            rrt_star.draw_graph(scan_list)
            plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
            plt.grid(True)
            plt.pause(0.01)  # Need for Mac
            plt.show()


if __name__ == '__main__':
    main()
            









