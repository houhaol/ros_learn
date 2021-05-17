#! /usr/bin/env python

import rospy
from std_srvs.srv import Empty, EmptyResponse
from move_turtle import moveTurtle

def my_callback(request):
    move_object = moveTurtle()
    move_object.move_square()
    rospy.loginfo("finished calling")
    return EmptyResponse

def main():
    rospy.init_node("move_servcie")
    rospy.Service("/move_turtle_in_squre", Empty, my_callback)    
    rospy.spin()


if __name__ == "__main__":
    main()