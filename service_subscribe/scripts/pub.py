#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

def main():
    
    rospy.init_node("pub_test")
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    rate = rospy.Rate(2)

    command = Twist()
    command.linear.x = 0
    command.angular.z = 0
    
    while not rospy.is_shutdown():
        pub.publish(command)
        rate.sleep()


if __name__ == "__main__":
    main()