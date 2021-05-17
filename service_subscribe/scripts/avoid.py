#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

global mid_dis; global min_dis

def scan_callback(msg):
    mid_dis = msg.ranges[len(msg.ranges)//2]
    min_dis = msg.range_min
    actions(mid_dis)

def actions(mid_dis):
    command = Twist()
    
    if mid_dis < 30:
        print("obstacles detected within 30m")
        command.angular.z = 0.1
        command.linear.x = 0
    else:
        print("No obstacles")
        command.angular.z = 0
        command.linear.x = 0.1
    pub.publish(command)

def main():
    global pub
    rospy.init_node("sub_test")
    sub = rospy.Subscriber("/scan", LaserScan, scan_callback)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    rospy.spin()
    
if __name__ == "__main__":
    main()