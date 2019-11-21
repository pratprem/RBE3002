#!/usr/bin/env python
from nav_msgs.srv import GetPlan
from geometry_msgs.msg import Pose, PoseStamped, Point
from nav_msgs.msg import Odometry
import rospy
from tf.transformations import euler_from_quaternion
class Lab3:

    def __init__(self):
        """
        Class constructor
        """
        ### Initialize node, name it 'lab2'
        rospy.init_node('lab3')
        #subscribe to target_node
        rospy.Subscriber('/move_base_simple/goal', PoseStamped , self.go_to)
        rospy.Subscriber('/odom', Odometry , self.update_odometry)

        #initiallizing variables needed
        self.px=0.0
        self.py=0.0
        self.pth=0.0

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
        #convert from rocketship numbers to english
        quat_orig = msg.pose.orientation
        quat_list = [quat_orig.x, quat_orig.y, quat_orig.z, quat_orig.w]
        (roll , pitch , yaw) = euler_from_quaternion (quat_list)
        #initial pose
        init_pose=PoseStamped(pose=Pose(position=Point(x=self.px,y=self.py)))
        #target pose
        target_pose=PoseStamped(pose=Pose(position=Point(x=msg.pose.position.x,y=msg.pose.position.y)))

        #call Service
        rospy.loginfo('Getting Path')
        rospy.wait_for_service('plan_path')
        service=rospy.ServiceProxy('plan_path',GetPlan)
        path=service(init_pose,target_pose,0.0)

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
    planner=Lab3()
    planner.run()
