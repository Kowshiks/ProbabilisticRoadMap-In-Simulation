from cmath import sin
from turtle import color
import matplotlib.pyplot as plt 
import matplotlib.animation as animation 
from matplotlib.patches import Rectangle
from matplotlib.patches import Polygon
import numpy as np 
import math
import sys
import time
import heapq
import collections
import multiprocessing


class valueRecord:
    
    def __init__(self):
        
        self.start_x = -45
        self.start_y = -60
        self.goal_x = 10
        self.goal_y = 60
        self.v = 0.3
        self.sample_size = 35
        self.obstacle_size = 5

        self.frames = 500
        

class obstacleBoundary:

    def __init__(self):

        self.start = [-45,-60]
        self.goal = [10,60]

        self.short_dist = float("inf")

class randomSample:

    def __init__(self):

        self.curValue = valueRecord()

        self.x = np.random.uniform(-70, 70, self.curValue.sample_size)
        self.y = np.random.uniform(-70, 70, self.curValue.sample_size)


        self.x = np.append(self.x, self.curValue.goal_x)
        self.y = np.append(self.y, self.curValue.goal_y)
    
    def val(self):

        return self.x,self.y


class obstacle:


    def __init__(self):

        self.all_lines = []

        self.all_lines.append([[-45,-40,-25,-40],[-25,-20,-25,-40],[-45,-20,-45,-40],[-45,-20,-25,-20]])
        self.all_lines.append([[-5,-20,15,-20],[15,-20,15,5],[15,5,-5,-20]])
        self.all_lines.append([[-40,-10,5,10],[5,10,-30,25],[-30,25,-40,-10]])
        self.all_lines.append([[20,40,40,40],[40,40,30,10],[30,10,20,40]])
        self.all_lines.append([[-25,35,-5,35],[-5,35,-5,50],[-5,50,-25,55],[-25,55,-25,35]])


    def val(self,index):

        if index == 1:

            pts = np.array([[-45,-40], [-45,-20], [-25,-20], [-25,-40]])
            p = Polygon(pts, fc='blue')

        if index == 2:

            pts = np.array([[-5,-20], [15,-20], [15,5]])
            p = Polygon(pts, fc='blue')

        if index == 3:

            pts = np.array([[-40,-10], [5,10], [-30,25]])
            p = Polygon(pts, fc='blue')

        if index == 4:

            pts = np.array([[30,10], [40,40], [20,40]])
            p = Polygon(pts, fc='blue')

        if index == 5:

            pts = np.array([[-25,35], [-25,55], [-5,50], [-5,35]])
            p = Polygon(pts, fc='blue')

        return p

    def lines(self):

        return self.all_lines


class CarSimulator:
    
    def __init__(self):
    
        self.curValue = valueRecord()
        self.obstacle = obstacleBoundary()
        self.curValue.time = time.time()
        self.sample = randomSample()
        self.demoObstalce = obstacle()

        self.final_path = []

        self.index = 0

        self.counter = 0

        self.timer = 0


        self.frame_dist_value = 0

        self.velocity = 0

    
    def shortestPath(self):

        shortest_path = []

        heap = []
        visited = []
        visited.append(self.obstacle.start)

        dist = math.sqrt((self.obstacle.goal[0] - self.obstacle.start[0])**2 + (self.obstacle.goal[1] - self.obstacle.start[1])**2)

        heapq.heappush(heap, [dist,self.obstacle.start,0,visited])

        all_visit = set()

        all_visit.add((self.obstacle.start[0],self.obstacle.start[1]))


        all_nodes = self.path_planning()

        while heap:

            d, cur_node, tot_dist, path = heapq.heappop(heap)

            if self.obstacle.short_dist > tot_dist:

                if (cur_node[0],cur_node[1]) in all_nodes.keys():

                    for each_node in all_nodes[(cur_node[0],cur_node[1])]:

                        if each_node == self.obstacle.goal:

                            cur_dist = tot_dist + math.sqrt((each_node[0] - cur_node[0])**2 + (each_node[1] - cur_node[1])**2)

                            if cur_dist < self.obstacle.short_dist:

                                cur_path = path.copy()

                                cur_path.append(each_node)

                                shortest_path = cur_path.copy()

                                self.obstacle.short_dist = cur_dist

                            

                        elif each_node not in path and (each_node[0],each_node[1]) not in path:

                            dist_goal = math.sqrt((self.obstacle.goal[0] - each_node[0])**2 + (self.obstacle.goal[1] - each_node[1])**2)

                            cur_path = path.copy()

                            cur_path.append(each_node)

                            cur_dist = tot_dist + math.sqrt((each_node[0] - cur_node[0])**2 + (each_node[1] - cur_node[1])**2)

                            heapq.heappush(heap, [dist_goal,each_node,cur_dist,cur_path])

                            all_visit.add((each_node[0],each_node[1]))

        return shortest_path

    
    # The reference for the below function is taken from : https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/
    def collision_check(self, p1, q1, p2, q2):

        # This function is used to check if 2 line segments intersect or not.

        class FindInter:
            def __init__(self, x_val, y_val):

                self.x = x_val
                self.y = y_val

        def findSeg(point_p, point_q, val):

            if ( (point_q.x <= max(point_p.x, val.x)) and (point_q.x >= min(point_p.x, val.x)) and
                (point_q.y <= max(point_p.y, val.y)) and (point_q.y >= min(point_p.y, val.y))):

                return True
            return False

        def angle_val(point_p, point_q, val):
            
            ang_val = (float(point_q.y - point_p.y) * (val.x - point_q.x)) - (float(point_q.x - point_p.x) * (val.y - point_q.y))

            if (ang_val > 0):
                return 1
            elif (ang_val < 0):
           
                return 2
            else:
                return 0

        
        def checkIntersection(point1_p,point1_q,point2_p,point2_q):
            
        
            angle_1 = angle_val(point1_p, point1_q, point2_p)

            angle_2 = angle_val(point1_p, point1_q, point2_q)

            angle_3 = angle_val(point2_p, point2_q, point1_p)

            angle_4 = angle_val(point2_p, point2_q, point1_q)

            if ((angle_1 != angle_2) and (angle_3 != angle_4)):
                return True

            if ((angle_1 == 0) and findSeg(point1_p, point2_p, point1_q)):
                return True

            if ((angle_2 == 0) and findSeg(point1_p, point2_q, point1_q)):
                return True

            if ((angle_3 == 0) and findSeg(point2_p, point1_p, point2_q)):
                return True

            if ((angle_4 == 0) and findSeg(point2_p, point1_q, point2_q)):
                return True

            return False

        all_obstacle = self.demoObstalce.lines()

        for each_fig in all_obstacle:

            for each_line in each_fig:

                point1_p = FindInter(p1, q1)
                point1_q = FindInter(p2, q2)
                point2_p = FindInter(each_line[0], each_line[1])
                point2_q = FindInter(each_line[2], each_line[3])

                if checkIntersection(point1_p, point1_q, point2_p, point2_q):

                    return False
        
        return True

    

    def path_planning(self):

        start = [-45,-60]
        end = [10,60]

        dict = {}

        x_points,y_points = self.sample.val()        

        fringe = collections.deque()

        fringe.append((start[0],start[1]))

        while fringe:

            (x1,x2) = fringe.popleft()

            if (x1,x2) not in dict.keys() and (x1 != end[0] and x2 != end[1]):

                dict[x1,x2] = []

                for i in range(self.curValue.sample_size+1):

                    if self.collision_check(x1,x2,x_points[i],y_points[i]) and (x_points[i] != x1 and y_points[i] != x2):

                        dict[x1,x2].append([x_points[i],y_points[i]])

                        fringe.append((x_points[i],y_points[i]))            
        return dict

    
    def frame_dist(self,shortestPath):

        total_distance  = 0
        segment_distance = []
        frames_segment = []
        final = 0
        

        for each_path in range(len(shortestPath)-1):

            cur_dist = math.sqrt((shortestPath[each_path+1][0]-shortestPath[each_path][0])**2+(shortestPath[each_path+1][1]-shortestPath[each_path][1])**2)
            total_distance +=cur_dist
            segment_distance.append(cur_dist)


        for each_seg in range(len(segment_distance)):

            final +=self.curValue.frames*(segment_distance[each_seg]/total_distance)
            frames_segment.append(final)    
        self.velocity = total_distance/self.curValue.frames


        self.frame_dist_value = frames_segment




    def motion(self):

        plt.style.use('dark_background')

        figure = plt.figure() 
        axis = plt.axes(xlim=(-70, 70), ylim=(-70, 70)) 
        plot, = axis.plot([], [], lw=2) 
        
        patch = plt.Rectangle((-45, -60), 4, 2, fc='r')

        x_val = [-45]
        y_val = [-60]

        def init(): 
            #patch.center = (5, 5)
            axis.add_patch(patch)
            return patch,

        def cossin(tan_val):

            cos_val = np.cos(tan_val)
            sin_val = np.sin(tan_val)

            return (cos_val,sin_val)


        def carMotion(each):

            if each == 499:

                time.sleep(5)
                sys.exit()


            if each>self.frame_dist_value[self.index]:

                self.index+=1

            tan_val = math.atan2((self.final_path[self.index+1][1]-self.final_path[self.index][1]),(self.final_path[self.index+1][0]-self.final_path[self.index][0]))

            cos_val,sin_val = cossin(tan_val)
                
            x_val.append(x_val[each] + cos_val*self.velocity)
            y_val.append(y_val[each]+  sin_val*self.velocity)

            #patch.center = (x_val[each], y_val[each])
            patch.angle = float(((tan_val*180)/math.pi))

            patch.set_xy([x_val[-1], y_val[-1]])
            return patch,



        for i in range(self.curValue.obstacle_size):

            p = self.demoObstalce.val(i+1)
            ax = plt.gca()
            ax.add_patch(p)
        
        plt.plot(-45, -60, marker=".", markersize=12)

        plt.plot(10, 60, marker=".", markersize=12)

        shortest_path = self.shortestPath()

        cur_dist = math.sqrt((shortest_path[1][0] - shortest_path[0][0])**2 + (shortest_path[1][1] - shortest_path[1][1])**2)

        self.timer = cur_dist*0.1


        self.final_path = shortest_path

        self.frame_dist(shortest_path)

        for path in range(len(shortest_path)-1):

            x, y = [shortest_path[path][0], shortest_path[path+1][0]], [shortest_path[path][1], shortest_path[path+1][1]]

            plt.plot(x,y,"lime")


        plt.title('PRM Simulator') 
        plt.axis('off') 

        graphicPlot = animation.FuncAnimation(figure, carMotion, init_func=init, frames=500, interval=10, blit=True) 

        plt.show()


def start():

    simulator = CarSimulator()

    simulator.motion()


if __name__ == "__main__":
    
    p = multiprocessing.Process(target=start, name="PRM Simulation")
    p.start()

    # Wait 10 seconds for foo
    time.sleep(30)

    print("If the samples couldnt find a path to the goal. Please run the program again")

    # Terminate foo
    p.terminate()


