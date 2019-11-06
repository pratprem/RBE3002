#!/usr/bin/env python2
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
import math

class Lab2:

    def __init__(self):
        """
        Class constructor
        """
        ### Initialize node, name it 'lab2'
        rospy.init_node('lab2')
        ### Tell ROS that this node publishes Twist messages on the '/cmd_vel' topic
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist)
        #subscribing to odom and pose nodes
        rospy.Subscriber('/odom', Odometry , self.update_odometry)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped , self.go_to)
        #initiallizing variables needed
        self.px=0.0
        self.py=0.0
        self.pth=0.0
        rospy.sleep(1)



    def send_speed(self, linear_speed=0, angular_speed=0):
        """
        Sends the speeds to the motors.
        :param linear_speed  [float] [m/s]   The forward linear speed.
        :param angular_speed [float] [rad/s] The angular speed for rotating around the body center.
        """
        # Create twist message
        msg_cmd_vel = Twist()
        # Linear velocity
        msg_cmd_vel.linear.x = linear_speed
        msg_cmd_vel.linear.y = 0.0
        msg_cmd_vel.linear.z = 0.0
        # Angular velocity
        msg_cmd_vel.angular.x = 0.0
        msg_cmd_vel.angular.y = 0.0
        msg_cmd_vel.angular.z = angular_speed
        # Send command
        self.cmd_vel.publish(msg_cmd_vel)



    def drive(self, distance, linear_speed=.1, tolerance=.05):
        """
        Drives the robot in a straight line.
        :param distance     [float] [m]   The distance to cover.
        :param linear_speed [float] [m/s] The forward linear speed.
        """
        #save initial pose
        init_pose={'x':self.px,'y':self.py}
        #useful variables
        cur_dist=0
        #drive forward speed
        self.send_speed(linear_speed)
        #while distance not in tolerance
        while abs(distance-cur_dist) > tolerance:
            #calculate cur distance
            cur_dist=math.hypot(self.px-init_pose['x'], self.py-init_pose['y'])
            #sleep cause too much speed
            rospy.sleep(.05)
        #send stop moving speed
        self.send_speed()



    def rotate(self, angle, aspeed=.1, tolerance=.05):
        """
        Rotates the robot around the body center by the given angle.
        :param angle         [float] [rad]   The distance to cover.
        :param angular_speed [float] [rad/s] The angular speed.
        """
        #save initial pose
        init_angle=self.pth
        #useful variables
        cur_rot=0
        #drive forward speed
        if angle>0:
            self.send_speed(angular_speed=abs(aspeed))
        else:
            self.send_speed(angular_speed=-abs(aspeed))
        #while not enough rotate
        while abs(angle-cur_rot) > tolerance:
            #calculate cur distance
            cur_rot=math.fmod(self.pth-init_angle,2*math.pi)
            #sleep cause too much speed
            rospy.sleep(.05)
        #send stop moving speed
        self.send_speed()

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
        init_pose={'x':self.px,'y':self.py, 'th':self.pth}
        #target pose
        target_pose={'x':msg.pose.position.x,'y':msg.pose.position.y, 'th':yaw}
        #calculate the l bowserjr needs to drive
        distance=math.hypot(target_pose['x']-init_pose['x'],target_pose['y']-init_pose['y'])
        #calculate the angle bowser needs to turn to so he faces driving direction
        alpha=math.atan2(target_pose['y']-init_pose['y'], target_pose['x']-init_pose['x'])
        #calculate the difference  in angles and feed that into Rotate
        self.rotate(alpha-init_pose['th'])
        #drive distance
        self.drive(distance)
        #rotate to face final direction
        self.rotate(target_pose['th']-alpha)


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



    def arc_to(self, position):
        """
        Drives to a given position in an arc.
        :param msg [PoseStamped] The target pose.
        """
        ### EXTRA CREDIT
        # TODO
        pass # delete this wwhile not rospy.is_shutdown():hen you implement your code



    def smooth_drive(self, distance, linear_speed):
        """
        Drives the robot in a straight line by changing the actual speed smoothly.
        :param distance     [float] [m]   The distance to cover.
        :param linear_speed [float] [m/s] The maximum forward linear speed.
        """
        ### EXTRA CREDIT
        # TODO
        pass # delete this when you implement your code



    def run(self):
        rospy.spin()

if __name__ == '__main__':
    while not rospy.is_shutdown():
        node=Lab2()
        node.run()
