#!/usr/bin/env python
import rospy
import sys
from map import Map
from edge import Edge
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped
from priority_queue import PriorityQueue
from std_msgs.msg import Bool

class Explorer:

    def __init__(self):
        """
        Class constructor
        """
        ### Initialize node, name it 'nav'
        rospy.init_node('explorer')
        #subscribe to target_node
        self.go=rospy.Publisher('/move_base_simple/goal', PoseStamped)
        self.frontier = rospy.Publisher('/path_planner/frontier',GridCells)
        self.centroids = rospy.Publisher('/path_planner/centroids',GridCells)
        self.request_map()
        #initiallizing variables needed
        self.px=0.0
        self.py=0.0
        self.pth=0.0

    def request_map(self):
        """
        Requests the map from the map server.
        :return [OccupancyGrid] The grid if the service call was successful,
                                None in case of error.
        """
        rospy.loginfo('Explorer: Getting Map')
        rospy.wait_for_service('dynamic_map')
        service=rospy.ServiceProxy('dynamic_map',GetMap)
        self.map=Map(service().map)
        rospy.loginfo('Explorer: Got Map')

    def main(self):
        while True:
            rospy.loginfo('Explorer: Calculating Frontiers')

            c_space=self.map.c_space()
            #turn map unkown into edges
            frontiers=c_space.isolate_frontiers()
            #expand and shrink edges
            dilate=frontiers.morph(1)
            erode=dilate.morph(-1)
            if erode:
                rospy.loginfo('Explorer: Finished Exploring!')
                self.map=None
                break
            #turn publish edges to frontier
            self.frontier.publish(erode.to_grid_cells())
            #turn eroded map to Edge object list
            edges=erode.to_edges()
            #add sort edges by size
            edges.sort(key=lambda e: 1/e.size)
            for e in edges:
                print(e)

            #send pose staped

            rospy.loginfo('Explorer: Sending Edge to Nav')
            rospy.loginfo(edges[0])

            self.go.publish(PoseStamped(pose=Pose(position=self.map.grid_to_world(edges[0].centroid))))
            rospy.loginfo('Explorer: Waiting for Robot to Drive')
            #wait for robot to go to path goal
            rospy.wait_for_message('/explorer/reached',Bool)
            rospy.loginfo('Explorer: Loop Complete Restarting')
            #update map after movement:
            self.request_map()

        rospy.loginfo("Finished Exploring")
    def update_odometry(self, msg):
        """
        Updates the current pose of the robot.
        This method is a callback bound to a Subscriber.
        :param msg [Odometry] The current odometry information.
        """
        self.px=msg.pose.pose.position.x
        self.py=msg.pose.pose.position.y
        quat_orig = msg.pose.pose.orientation
        quat_list = [quat_orig.x, quat_orig.y, quat_orig.z, quat_orig.w]
        (roll , pitch , yaw) = euler_from_quaternion (quat_list)
        self.pth = yaw

    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        rospy.spin()

if __name__ == '__main__':
    planner=Explorer()
    planner.main()
    planner.run()
