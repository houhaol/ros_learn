#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Point, Quaternion
import time
from nav_msgs.msg import Odometry
import tf
from math import radians
from angles import normalize_angle
from tf.transformations import euler_from_quaternion
from std_srvs.srv import Empty, EmptyRequest
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose


class moveTurtle():
    def __init__(self):
        self.turtle_vel_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.ctrl_c = False
        self.rate = rospy.Rate(10)
        rospy.on_shutdown(self.shutdown_signal)

        self.base_frame = '/base_footprint'
        self.odom_frame = '/odom'

        # Initialize the tf listener
        self.tf_listener = tf.TransformListener()
        
        self.amcl_pose_sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.amcl_pose_callback)
        self.amcl_msg = PoseWithCovarianceStamped()
        

    def shutdown_signal(self):
        self.stop_move()
        self.ctrl_c = True
    
    def publish_once_in_cmd_vel(self, cmd):
        while not self.ctrl_c:
            connections = self.turtle_vel_publisher.get_num_connections()
            if connections >0:
                self.turtle_vel_publisher.publish(cmd)
                rospy.loginfo("cmd Published")
                break
            else:
                self.rate.sleep()

    def stop_move(self):
        cmd = Twist()
        cmd.linear.x = 0
        cmd.angular.z = 0
        self.publish_once_in_cmd_vel(cmd)
        rospy.loginfo("stopped")

    def move(self, duration, linear_speed, angular_speed):
        cmd = Twist()
        cmd.linear.x = linear_speed
        cmd.linear.z = angular_speed

        rospy.loginfo("Moving initilization")
        self.publish_once_in_cmd_vel(cmd)
        time.sleep(duration)
        self.stop_move()
        rospy.loginfo("Finish moving")
    
    def get_odom(self):
        # Get the current transform between the odom and base frames
        try:
            (trans, rot)  = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return
        rotation_tmp = Quaternion(*rot)
        orientation_list = [rotation_tmp.x, rotation_tmp.y, rotation_tmp.z, rotation_tmp.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)

        return yaw 

    def turn(self):
        rotation = self.get_odom()
        cmd = Twist()
        cmd.angular.z = 0.2
        last_angle = rotation
        turn_angle = 0
        while abs(turn_angle + radians(2) < abs(radians(90))) and not self.ctrl_c:
            self.publish_once_in_cmd_vel(cmd)
            self.rate.sleep()

            rotation = self.get_odom()

            delta_angle = normalize_angle(rotation - last_angle)
                
            turn_angle += delta_angle
            last_angle = rotation

        self.stop_move()
        rospy.loginfo("finish turn")

    def move_square(self, side=0.2):
        i = 0
        duration = side/0.2
        position = Point()
        
        while not self.ctrl_c and i<4:
            # move
            self.move(duration=duration*2, linear_speed=0.2, angular_speed=0)
            #stop
            self.move(duration=duration, linear_speed=0, angular_speed=0)
            # turn
            rospy.loginfo("Star Turn")
            self.turn()
            # stop
            self.move(duration=duration, linear_speed=0, angular_speed=0)

            i += 1
        rospy.loginfo("finished moving squre")
        
    def disperse_call(self):
        rospy.loginfo("calling particles, disperse free space")
        particles_client = rospy.ServiceProxy("/global_localization", Empty)
        particles_request = EmptyRequest()
        particles_client(particles_request)

    def cov_calculate(self):
        rospy.loginfo("calculating covariance")
        var_x = self.amcl_msg.pose.covariance[0]
        var_y = self.amcl_msg.pose.covariance[7]
        var_z = self.amcl_msg.pose.covariance[35]
        return (var_x+var_y+var_z)/3


    def amcl_pose_callback(self, msg):
        self.amcl_msg = msg



if __name__ == "__main__":
    rospy.init_node("moveTurtle_amcl")
    move_object = moveTurtle()
    cov = 1
    while cov > 0.65:
        # do some stuff here
        # 1. call service to disperse the particiles
        move_object.disperse_call()

        # 2. square movement
        move_object.move_square(0.8)

        # 3. calculate covariance
        cov = move_object.cov_calculate()

        # 4. check cov value
        if cov > 0.65:
            rospy.loginfo("repeat the process")
        else:
            rospy.loginfo("less than 0.65, exiting.")