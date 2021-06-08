#! /usr/bin/python

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist, Point
from math import atan2


x = 0.0
y = 0.0
yaw = 0.0

def odom_cb(msg):
    global x
    global y
    global yaw
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    rot_q = msg.pose.pose.orientation
    (_, _, yaw) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w]) 

def execute(goal):
    rate = rospy.Rate(4)
    cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    cmd = Twist()
    while not rospy.is_shutdown():
        x_diff = goal.x - x
        y_diff = goal.y - y
        angle_diff = atan2(y_diff, x_diff)
        if abs(angle_diff - yaw) > 0.1:
            cmd.angular.z = 0.3
            cmd.linear.x = 0.0
        elif abs(x_diff) >0.01:
            cmd.angular.z = 0.0
            cmd.linear.x = 0.5
        else:
            cmd.angular.z = 0.0
            cmd.linear.x = 0.0
        cmd_pub.publish(cmd)
        rate.sleep()

def main():
    rospy.init_node("simple_controller")
    odom_sub = rospy.Subscriber("/odom", Odometry, odom_cb)
    goal = Point()
    goal.x = 5
    goal.y = 5
    execute(goal)

if __name__ == "__main__":
    main()