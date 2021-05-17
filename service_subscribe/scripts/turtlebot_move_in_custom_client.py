#! /usr/bin/env python

import rospy
from test.srv import turtleCustomServiceMessage, turtleCustomServiceMessageRequest


def main():
    rospy.init_node("move_client")
    rospy.wait_for_service("/move_turtle_in_squre")
    turtlebot_move_in_square_client = rospy.ServiceProxy("/move_turtle_in_squre", turtleCustomServiceMessage)
    turtlebot_move_in_square_object = turtleCustomServiceMessageRequest()

    for i in range(3):
        rospy.loginfo("start")
        turtlebot_move_in_square_object.side = (i+1)*2
        turtlebot_move_in_square_object.repetitions = i
        turtlebot_move_in_square_client(turtlebot_move_in_square_object)
    rospy.loginfo("End of service call")

if __name__ == "__main__":
    main()