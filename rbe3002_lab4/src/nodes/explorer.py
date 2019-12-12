import rospy
from ....util.map import Map
from ....util.edge import Edge
from nav_msgs.srv import GetPlan, GetMap
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Pose, PoseStamped
from ....util.priority_queue import PriorityQueue
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
        rospy.loginfo('Getting Map')
        rospy.wait_for_service('map')
        service=rospy.ServiceProxy('map',GetMap)
        self.map=Map(service().map)

    def main(self):
        while self.map:
            #turn map unkown into edges
            invert=self.map.invert()
            #find frontiers
            edged=invert.edge_detector()
            #expand and shrink edges
            dilate=edged.morph(2)
            erode=dilate.morph(-2)
            if erode:
                self.map=None
                break
            #turn publish edges to frontier
            self.frontier.publish(erode.to_grid_cells())
            #turn eroded map to Edge object list
            edges=erode.to_edges()
            queue=PriorityQueue()
            #add edges to queue
            for edge in edges:
                priority=abs(edge.size/self.map.euclidean_distance((self.px,self.py), edge.centroid))
                queue.put((priority,edge.centroid))
            #get top msg
            target=queue.pop()
            #send pose staped
            self.go.publish(PoseStamped(pose=Pose(position=self.map.grid_to_world(target))))
            #wait for robot to go to path goal
            rospy.wait_for_message('/explorer/reached',Bool)            
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
    planner=Explorer().main()
    planner.run()