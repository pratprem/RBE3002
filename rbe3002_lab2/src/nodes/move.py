#!/usr/bin/env python2
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import math
import numpy as np
from tf.transformations import euler_from_quaternion
import tf

class Move:

    def __init__(self):
        """
        Class constructor
        """
        ### Initialize node, name it 'map'
        rospy.init_node('map')
        ### Tell ROS that this node publishes Twist messages on the '/cmd_vel' topic
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist)
        self.reached = rospy.Publisher('path_exec/reached', Bool)
        #subscribing to odom and pose nodes
        rospy.Subscriber('/path_exec/go', PoseStamped , self.go_to)

        #init tfLinstener
        self.tfListener=tf.TransformListener()
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



    def drive(self, distance, linear_speed=.15, tolerance=.02):
        """
        Drives the robot in a straight line.
        :param distance     [float] [m]   The distance to cover.
        :param linear_speed [float] [m/s] The forward linear speed.
        :param tolerance    [float] [none] tolerance at which robot stops
        """
        #save initial pose
        self.update_odometry()
        init_pose={'x':self.px,'y':self.py}
        #useful variables
        cur_dist=0
        #drive forward speed
        self.send_speed(linear_speed)
        #while distance not in tolerance
        while abs(distance-cur_dist) > tolerance:
            #calculate cur distance
            self.update_odometry()
            cur_dist=math.hypot(self.px-init_pose['x'], self.py-init_pose['y'])
            #sleep cause too much speed
            rospy.sleep(.05)
        #send stop moving speed
        self.send_speed()



    def rotate(self, angle, aspeed=.2, tolerance=.04):
        """
        Rotates the robot around the body center by the given angle.
        :param angle         [float] [rad]   The distance to cover.
        :param angular_speed [float] [rad/s] The angular speed.
        :param tolerance    [float] [none] tolerance at which robot stops
        """
        self.update_odometry()
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
        while abs(self._dif_angle(angle,cur_rot))  > tolerance:
            #calculate cur distance
            self.update_odometry()
            cur_rot=self._dif_angle(self.pth,init_angle)
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
        #initial pose
        self.update_odometry()
        init_pose={'x':self.px,'y':self.py, 'th':self.pth}
        #target pose
        target_pose={'x':msg.pose.position.x,'y':msg.pose.position.y}
        #calculate the l bowserjr needs to drive
        distance=math.hypot(target_pose['x']-init_pose['x'],target_pose['y']-init_pose['y'])
        #calculate the angle bowser needs to turn to so he faces driving direction
        alpha=math.atan2(target_pose['y']-init_pose['y'], target_pose['x']-init_pose['x'])
        #calculate the difference  in angles and feed that into Rotate
        self.rotate(self._dif_angle(alpha,init_pose['th']))
        #drive distance
        self.drive(distance)
        self.reached.publish(Bool(True))

    def update_odometry(self):
        """
        Updates the current pose of the robot.
        This method is a callback bound to a Subscriber.
        :param msg [Odometry] The current odometry information.
        """
        (trans,rot)=self.tfListener.lookupTransform('/map','/base_footprint',rospy.Time(0))

        self.px=trans[0]
        self.py=trans[1]
        (roll , pitch , yaw) = euler_from_quaternion (rot)
        self.pth = yaw



    def arc_to(self, position):
        """
        Drives to a given position in an arc.
        :param msg [PoseStamped] The target pose.
        """
        ### EXTRA CREDIT
        # TODO
        pass # delete this wwhile not rospy.is_shutdown():hen you implement your code



    def smooth_drive(self, distance, linear_speed=.35, accel_distance=.1,tolerance=.05):
        """
        Drives the robot in a straight line by changing the actual speed smoothly.
        :param distance         [float] [m]   The distance to cover.
        :param linear_speed     [float] [m/s] The maximum forward linear speed.
        :param accel_distance   [float] [none] max precentage of distance to cover you want to accelerate for
        :param tolerance    [float] [none] tolerance at which robot stops
        """
        #save initial pose
        init_pose={'x':self.px,'y':self.py}
        #useful variables
        cur_dist=0
        sleep_time=.05
        #set initial speed to zero
        speed=0
        #generate aceeleration curve
        curve=self.curve_generator(int(accel_distance*distance*1000),scalar=linear_speed)
        #accelerate
        for speed in curve:
            if abs(distance-cur_dist) > tolerance and cur_dist < distance*accel_distance:
                break
            #send appropriate speed
            self.send_speed(speed)
            #calculate cur distance
            cur_dist=math.hypot(self.px-init_pose['x'], self.py-init_pose['y'])
            #sleep cause too much speed
            rospy.sleep(sleep_time)
        #set full speed
        self.send_speed(linear_speed)
        #reverse acceleration curve
        curve.reverse()
        #max speed
        while abs(distance-cur_dist) > tolerance and cur_dist < distance*(1-accel_distance):
            #calculate cur distance
            cur_dist=math.hypot(self.px-init_pose['x'], self.py-init_pose['y'])
            #sleep cause too much speed
            rospy.sleep(sleep_time)
        #decelerate
        for speed in curve:
            if abs(distance-cur_dist) > tolerance:
                break
            #send appropriate speed
            self.send_speed(speed)
            #calculate cur distance
            cur_dist=math.hypot(self.px-init_pose['x'], self.py-init_pose['y'])
            #sleep cause too much speed
            rospy.sleep(sleep_time)
        #send stop moving speed
        self.send_speed()

    def curve_generator(self,length,exponent=2, start=0, end=None, scalar=1, invert=False):
        """
        internal function that returns an acceleration curve with the desired amount of points
        :param  length      [int]   [none] number of integers you want in your curve
        :param  exponent    [float] [none] optional float that deterines the shape of the curve <1 is exponetial >1 is logaritmic 1 is linear
        :param  start       [int]   [none] optional scalar to set starting value of curve
        :param  end         [int]   [none] optional scalar for ending point of curver
        :param  scalar      [int]   [none] optional scalar to multiply curve by
        :param  invert      [int]   [none] optional boolean that makes the function return an inverted list
        :return curve       [list] [floats] retur
        """
        #if no end value specified set end to 1 above start
        if end == None: end=start+1
        #genrate curve
        curve=[scalar*(x**exponent) for x in np.linspace(start,end,length,endpoint=False)]
        #invert curve if invert is true
        if invert:
            curve.reverse()
        #return
        return curve



    def _norm(self,*args):
        """
        internal functions to wrap all angles into -pi to pi space
        :param  args        [float] [radians] an list of angles to wrap
        :return angles      [float] [radians] list of angles returned -pi to pi
        """
        #convert angle to 0 to 2pi space
        r= [math.fmod(math.fmod(a,2*math.pi) +2*math.pi,2*math.pi) for a in args]
        #convert from wrap angle form pi to 2pi to -pi to zero
        result = tuple([a-2*math.pi if a>math.pi else a for a in r])

        #if len of result is 1 unpack tuple else dont touch
        if len(result)==1:
            return result[0]
        else:
            return result
        return result

    def _dif_angle(self,a,b):
        """
        internal functions to subtract angle a from angle b
        :param  a        [float] [radians] first angle
        :param  b        [float] [radians] second angle
        :return answer   [float] [radians] a-b
        """
        #normalize a and be subtract the result and normalize it then return
        a,b=self._norm(a,b)
        return self._norm(a-b)


    def run(self):
        rospy.spin()

if __name__ == '__main__':
    while not rospy.is_shutdown():
        node=Move()
        node.run()
