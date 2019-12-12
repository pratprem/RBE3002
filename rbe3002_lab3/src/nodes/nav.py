#!/usr/bin/env python
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import Pose, PoseStamped, Point
from nav_msgs.msg import Odometry , Path
from std_msgs.msg import Bool
import rospy
from tf.transformations import euler_from_quaternion
class Nav:

    def __init__(self):
        """
        Class constructor
        """
        ### Initialize node, name it 'nav'
        rospy.init_node('nav')
        #subscribe to target_node
        rospy.Subscriber('/move_base_simple/goal', PoseStamped , self.go_to)
        rospy.Subscriber('/odom', Odometry , self.update_odometry)
        self.path_pub=rospy.Publisher('/path_planner/path', Path)
        self.robot_go=rospy.Publisher('/path_exec/go', PoseStamped)
        self.finish_path=rospy.Publisher('/explorer/reached',Bool)
        #initiallizing variables needed
        self.px=0.0
        self.py=0.0
        self.pth=0.0
        self.ready=True
        rospy.sleep(1)

    def run(self):
        """
        Runs the node until Ctrl-C is pressed.
        """
        rospy.spin()


    def go_to(self, msg):
        """
        Calls rotate(), drive(), and rotate() to attain a given pose.
        This method is a callback bound to a Subscriber.
        :param msg [PoseStamped] The target pose.
        """
        #initial pose
        init_pose=PoseStamped(pose=Pose(position=Point(x=self.px,y=self.py)))
        #target pose
        target_pose=PoseStamped(pose=Pose(position=Point(x=msg.pose.position.x,y=msg.pose.position.y)))

        #call Service
        rospy.loginfo('Getting Path')
        rospy.wait_for_service('plan_path')
        service=rospy.ServiceProxy('plan_path',GetPlan)
        path=service(init_pose,target_pose,1.0).plan
        self.path_pub.publish(path)
        for pose in path.poses:
            self.robot_go.publish(pose)
            rospy.wait_for_message('/path_exec/reached',Bool)
        self.finish_path.pulish(Bool(True))


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

if __name__ == '__main__':
    planner=Nav()
    planner.run()
