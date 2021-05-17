#! /usr/bin/env python

import rospy
from std_srvs.srv import Empty, EmptyRequest


def main():
    rospy.init_node("move_client")
    rospy.wait_for_service("/move_turtle_in_squre")
    turtlebot_move_in_square_client = rospy.ServiceProxy("/move_turtle_in_squre", Empty)
    turtlebot_move_in_square_object = EmptyRequest()
    turtlebot_move_in_square_client(turtlebot_move_in_square_object)

if __name__ == "__main__":
    main()