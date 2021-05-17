#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan

def scan_callback(msg):
    print(msg.range_min)


def main():
    rospy.init_node("sub_test")
    rospy.Subscriber("/scan", LaserScan, scan_callback)

    rospy.spin()

if __name__ == "__main__":
    main()