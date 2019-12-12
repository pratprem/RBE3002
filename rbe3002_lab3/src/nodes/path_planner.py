#!/usr/bin/env python
import math
import rospy
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped

from map import Map
from priority_queue import PriorityQueue

class PathPlanner:

    def __init__(self):
        """
        Class constructor
        """
        ### REQUIRED CREDIT
        ## Initialize the node and call it "path_planner"
        rospy.init_node('path_planner')
        ## Create a new service called "plan_path" that accepts messages of
        ## type GetPlan and calls self.plan_path() when a message is received
        self.path_serv=rospy.Service('plan_path',GetPlan,self.plan_path)
        ## Create a publisher for the C-space (the enlarged occupancy grid)
        ## The topic is "/path_planner/cspace", the message type is GridCells
        self.cspace = rospy.Publisher('/path_planner/cspace', GridCells)
        ## Create publishers for A* (expanded cells, frontier, ...)
        ## Choose a the topic names, the message type is GridCells
        self.explored_cells = rospy.Publisher('/path_planner/explored_cells',GridCells)
        ## Sleep to allow roscore to do some housekeeping
        rospy.sleep(1.0)
        rospy.loginfo("Path planner node ready")

    @staticmethod
    def request_map():
        """
        Requests the map from the map server.
        :return [OccupancyGrid] The grid if the service call was successful,
                                None in case of error.
        """
        rospy.loginfo('Getting Map')
        rospy.wait_for_service('dynamic_map')
        service=rospy.ServiceProxy('dynamic_map',GetMap)
        return service().map

    def calc_cspace(self, map, padding):
        """
        Calculates the C-Space, i.e., makes the obstacles in the map thicker.
        Publishes the list of cells that were added to the original map.
        :param map [Map]           The map object.
        :param padding [int]           The number of cells around the obstacles.
        :return        [[int8]]        The C-Space as a list of values from 0 to 100.
        """
        rospy.loginfo("Calculating C-Space")
        new_map=map.c_space(padding,path=True)
        self.cspace.publish(new_map.to_grid_cells())
        rospy.loginfo("Calculated C-Space")
        return new_map

    def a_star(self, map, start, goal):
        rospy.loginfo("Executing A* from (%d,%d) to (%d,%d)" % (start[0], start[1], goal[0], goal[1]))
        queue=PriorityQueue(start)
        visited=set()
        came_from={}
        while queue:
            #self.yeet(map, visited, queue.get_elements())
            element, previous_element=queue.pop()
            visited.add(element)
            came_from[element]=previous_element
            if element == goal:
                path=[element]
                while came_from[element]:
                    element=came_from[element]
                    path.append(element)
                return path[::-1]
            [queue.put((map.euclidean_distance(start,e)+ map.euclidean_distance(e,goal) + 100*map.euclidean_distance(e,element),e,element)) for e in map.get_neighbors(element,threshold2=0) if e not in visited and e not in queue.get_elements()]

    def yeet(self, map ,visited, queue):
        """
        yeets visited and queue on the appropriate topics
        i'm tired and have to be up at 8am :(
        """
        self.explored_cells.publish(map.point_to_grid_cells(visited))

    @staticmethod
    def optimize_path(path):
        """
        Optimizes the path, removing unnecessary intermediate nodes.
        :param path [[(x,y)]] The path as a list of tuples (grid coordinates)
        :return     [[(x,y)]] The optimized path as a list of tuples (grid coordinates)
        """
        rospy.loginfo("Optimizing path")
        i=0
        print(path)
        while len(path) >= 3 and i<len(path)-2:
            if PathPlanner.colinear(*path[i:i+3]):
                del path[i+1]
            else:
                i+=1
        return path


    @staticmethod
    #determines if 3 points are colinear
    def colinear(a,b,c):
        return (c[1]-b[1])*(b[0]-a[0]) == (b[1]-a[1])*(c[0]-b[0])

    def path_to_message(self, map, path):
        """
        Takes a path on the grid and returns a Path message.
        :param path [[(int,int)]] The path on the grid (a list of tuples)
        :return     [Path]        A Path message (the coordinates are expressed in the world)
        """
        rospy.loginfo("Returning a Path message")
        msg=Path()
        msg.header.frame_id='map'
        msg.poses=[PoseStamped(pose=Pose(position=map.grid_to_world(loc))) for loc in path]
        return msg

    def plan_path(self, msg):
        """
        Plans a path between the start and goal locations in the requested.
        Internally uses A* to plan the optimal path.
        :param req
        """
        ## Request the map
        ## In case of error, return an empty path
        map = Map(PathPlanner.request_map())
        if map is None:
            return Path()
        ## Calculate the C-space and publish it
        cspace_map = self.calc_cspace(map, 3)
        ## Execute A*
        start = cspace_map.world_to_grid(msg.start.pose.position)
        goal  = cspace_map.world_to_grid(msg.goal.pose.position)
        path  = self.a_star(cspace_map, cspace_map.nearest_walkable_neigbor(start), cspace_map.nearest_walkable_neigbor(goal))
        ## Optimize waypoints
        waypoints = PathPlanner.optimize_path(path)
        ## Return a Path message
        return self.path_to_message(map, waypoints)

    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        rospy.spin()

if __name__ == '__main__':
    planner=PathPlanner()
    planner.run()
